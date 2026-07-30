/*
 *   Copyright (C) 2026 by Tom Fanning M0LTE
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

/*
 * Mode 3: 19200 bps C4FSK IL2P.
 *
 * The same waveform as mode 2 at twice the symbol rate: 9600 symbols per
 * second, five samples per symbol at a 48 kHz sample clock. Confirmed against
 * both pymodem (whose IL2P codec searches for the same 0x5D57DF7F sync for
 * every modem) and the NinoTNC 19200 capture, which matches mode 2's sync
 * template with 0 of 16 symbols wrong. The Mode3 sources are generated from
 * the fixed Mode2 ones, so every receiver fix carries over.
 *
 * The upstream mode3 branch predates the August 2024 interoperability work
 * and still carries the old six byte sync vector; it would not have decoded
 * a NinoTNC at all.
 */

#include "Config.h"
#include "Globals.h"

#include "Mode3Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>

namespace {

  const char* CAPTURE_19200 = "../testdata/ninotnc-v44-19200-C4FSK-IL2Pc-0001.wav";
  const char* BULK_19200    = "../testdata/ninotnc-v44-19200-C4FSK-IL2Pc-0001-100-frames.wav";

  const int NOMINAL_PEAK = 695;

  /* 48 kHz capture, 48 kHz receiver: no decimation. */
  std::vector<uint16_t> capture(const char* path)
  {
    unsigned rate = 0U;
    const std::vector<int16_t> pcm = radio::loadWav(path, rate);

    if (pcm.empty() || rate != 48000U)
      return std::vector<uint16_t>();

    return radio::toAdc(pcm, NOMINAL_PEAK);
  }

  unsigned beaconsDecoded(const std::vector<std::vector<uint8_t> >& frames)
  {
    std::set<int> seen;

    for (size_t i = 0U; i < frames.size(); i++) {
      const std::string body(frames[i].begin() + 1, frames[i].end());
      const size_t at = body.find("Beacon from M0LTE");
      if (at != std::string::npos)
        seen.insert(std::atoi(body.c_str() + at + 17));
    }

    return unsigned(seen.size());
  }

}

TF_TEST(mode3_uses_the_interoperable_sync_vector)
{
  /* The wire format decision, pinned down: same sync as mode 2, not the
     six byte one from the stale upstream branch. */
  REQUIRE_EQ(int(MODE3_SYNC_LENGTH_BYTES), 4);
  CHECK_EQ(int(MODE3_SYNC_BYTES[0]), 0x5D);
  CHECK_EQ(int(MODE3_SYNC_BYTES[1]), 0x57);
  CHECK_EQ(int(MODE3_SYNC_BYTES[2]), 0xDF);
  CHECK_EQ(int(MODE3_SYNC_BYTES[3]), 0x7F);
}

TF_TEST(mode3_selects_the_48k_sample_clock)
{
  /* Switching to mode 3 must retune the sample clock, and switching back must
     restore it. */
  m_mode = 3U;
  io.showMode();
  CHECK_EQ(int(hooks::g_sampleRate), 48000);

  m_mode = 2U;
  io.showMode();
  CHECK_EQ(int(hooks::g_sampleRate), 24000);
}

TF_TEST(mode3_decodes_its_own_transmitter)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  const std::vector<uint16_t> wave = radio::modulate(&payload[0], uint16_t(payload.size()), 3U, 3U);
  REQUIRE_MSG(wave.size() > 1000U, "transmitter produced only " << wave.size() << " samples");

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(wave, true, 3U);

  CHECK_MSG(radio::decodedExactly(frames, payload),
            "a noiseless mode 3 packet did not decode; got " << frames.size() << " frame(s)");
}

TF_TEST(mode3_decodes_at_every_sampling_phase_and_both_senses)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> clean = radio::modulate(&payload[0], uint16_t(payload.size()), 3U, 3U);

  for (int inverted = 0; inverted < 2; inverted++) {
    for (unsigned phase = 0U; phase < MODE3_RADIO_SYMBOL_LENGTH; phase++) {
      radio::Channel ch;
      ch.phase  = phase;
      ch.invert = (inverted != 0);

      const std::vector<std::vector<uint8_t> > frames =
        radio::demodulate(radio::applyChannel(clean, ch), true, 3U);

      CHECK_MSG(radio::decodedExactly(frames, payload),
                "lost the packet at phase " << phase
                << " with deviation " << (inverted ? "inverted" : "normal"));
    }
  }
}

TF_TEST(mode3_ninotnc_sync_and_header_decode)
{
  /* Ten transmissions from a real NinoTNC in its 19200 C4FSK IL2Pc mode. */
  const std::vector<uint16_t> adc = capture(CAPTURE_19200);
  REQUIRE_MSG(!adc.empty(), "could not read " << CAPTURE_19200);

  radio::demodulate(adc, true, 3U);

  unsigned syncs = 0U, headers = 0U;
  const std::string& d = hooks::g_debugTx;
  for (size_t p = d.find("sync found"); p != std::string::npos; p = d.find("sync found", p + 1U))
    syncs++;
  for (size_t p = d.find("header is valid"); p != std::string::npos; p = d.find("header is valid", p + 1U))
    headers++;

  CHECK_MSG(syncs   >= 10U, "only " << syncs   << " of 10 transmissions produced a sync");
  CHECK_MSG(headers >= 10U, "only " << headers << " of 10 headers decoded");
}

TF_TEST(mode3_ninotnc_bulk_decode_rate)
{
  const std::vector<uint16_t> adc = capture(BULK_19200);
  REQUIRE_MSG(!adc.empty(), "could not read " << BULK_19200);

  const unsigned got = beaconsDecoded(radio::demodulate(adc, true, 3U));

  std::printf("      decoded %u of 100 beacons\n", got);

  /* Raise this as the receiver improves. The target is 100. */
  CHECK_MSG(got >= 81U,
            "decoded " << got << " of 100 beacons from the 19200 bulk capture; the floor is 81");
}

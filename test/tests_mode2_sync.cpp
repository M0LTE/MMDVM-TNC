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
 * Mode 2 sync acquisition.
 *
 * The receiver builds a 16 bit history of symbol signs, one entry per sample
 * phase, and looks for MODE2_SYNC_SYMBOLS in it. Because the newest sample's
 * sign is shifted in at bit 0 before correlateSync() runs, a match means sync
 * symbol S0 sits at m_dataPtr - 75 and S15 sits at m_dataPtr itself. Every
 * window the correlator opens has to line up with that.
 */

#include "Config.h"
#include "Globals.h"

#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

TF_TEST(mode2_decodes_a_clean_packet)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  radio::Channel ch;
  const std::vector<std::vector<uint8_t> > frames = radio::loopback(payload, ch);

  CHECK_MSG(radio::decodedExactly(frames, payload),
            "a noiseless packet from this firmware's own transmitter did not decode; "
            "got " << frames.size() << " frame(s)");
}

TF_TEST(mode2_decodes_at_every_sampling_phase)
{
  /*
   * Five samples per symbol, so a whole-sample delay of 0..4 sweeps the
   * receiver's sampling instant across a complete symbol period. The
   * transmitter's and receiver's sample clocks are independent and drift, so
   * on the air the phase is whatever it happens to be when the packet
   * arrives. All five have to work or reception is a lottery.
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> clean = radio::modulate(&payload[0], uint16_t(payload.size()));

  unsigned decoded = 0U;

  for (unsigned phase = 0U; phase < MODE2_RADIO_SYMBOL_LENGTH; phase++) {
    radio::Channel ch;
    ch.phase = phase;

    const std::vector<std::vector<uint8_t> > frames = radio::demodulate(radio::applyChannel(clean, ch));

    const bool ok = radio::decodedExactly(frames, payload);
    if (ok)
      decoded++;

    CHECK_MSG(ok, "lost the packet at sampling phase " << phase);
  }

  CHECK_EQ(int(decoded), int(MODE2_RADIO_SYMBOL_LENGTH));
}

TF_TEST(mode2_decodes_both_deviation_senses)
{
  /*
   * From the README: "there is no correct way in which the deviation is
   * decoded and so the receive side is able to detect and decode
   * transmissions of either sense."
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> clean = radio::modulate(&payload[0], uint16_t(payload.size()));

  for (int inverted = 0; inverted < 2; inverted++) {
    for (unsigned phase = 0U; phase < MODE2_RADIO_SYMBOL_LENGTH; phase++) {
      radio::Channel ch;
      ch.phase  = phase;
      ch.invert = (inverted != 0);

      const std::vector<std::vector<uint8_t> > frames = radio::demodulate(radio::applyChannel(clean, ch));

      CHECK_MSG(radio::decodedExactly(frames, payload),
                "lost the packet at phase " << phase
                << " with deviation " << (inverted ? "inverted" : "normal"));
    }
  }
}

TF_TEST(mode2_decodes_the_same_packet_played_on_repeat)
{
  /*
   * The reported symptom, reproduced: the same frame played into the modem
   * over and over, a few decoding and a few going missing. The sampling phase
   * walks between bursts, which is what two free-running sample clocks do.
   */
  const unsigned REPEATS = 10U;

  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> clean = radio::modulate(&payload[0], uint16_t(payload.size()));

  std::vector<uint16_t> stream;

  for (unsigned i = 0U; i < REPEATS; i++) {
    radio::Channel ch;
    ch.phase = i % MODE2_RADIO_SYMBOL_LENGTH;

    radio::append(stream, radio::applyChannel(clean, ch));
    radio::append(stream, radio::silence(2400U));      /* 100 ms between bursts */
  }

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(stream);

  unsigned good = 0U;
  for (size_t i = 0U; i < frames.size(); i++) {
    if (frames[i].size() == payload.size() + 1U &&
        ::memcmp(&frames[i][1], &payload[0], payload.size()) == 0)
      good++;
  }

  CHECK_MSG(good == REPEATS,
            "played the same packet " << REPEATS << " times, decoded " << good);
}

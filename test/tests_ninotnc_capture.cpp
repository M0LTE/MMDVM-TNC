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
 * Receiving a real NinoTNC.
 *
 * Every other test in this suite drives this firmware's own transmitter into
 * its own receiver. That catches a receiver which disagrees with itself, but
 * it is structurally blind to anything the two agree on and the rest of the
 * world does not. These captures are off a NinoTNC running v3/4.44 and share
 * no assumption with this repository.
 *
 * Mode 2's waveform is Nino Carrillo's: he tuned the filter taps, sync vector,
 * SCALING_FACTOR and sync tolerance in g4klx/MMDVM-TNC PRs #3 and #4 between
 * 31 July and 2 August 2024, the same week NinoTNC firmware v3/4.40 added its
 * 9600 and 19200 bps C4FSK IL2Pc modes. The two are meant to interoperate.
 *
 * Captures made with tnc-tools kiss-ax25-ui-batch.py: ten 100 byte AX.25 UI
 * frames carrying "Beacon from M0LTE" with an incrementing counter.
 *
 * WHAT THIS ESTABLISHES
 *
 * This firmware decodes a real NinoTNC off air, end to end.
 *
 * These captures replaced an earlier pair made with AGC enabled on the
 * recorder. That AGC left the preamble and sync intact -- they are all +/-3
 * symbols, so a compressor has nothing to act on -- and the 15 byte header
 * survived too, but it wrecked 116 bytes of four level payload. Nino
 * Carrillo's own decoder, pymodem, failed on those recordings in exactly the
 * same place, which is what identified the recorder rather than the firmware.
 *
 * Interoperability is now demonstrated in both directions:
 *
 *   - NinoTNC to here: sync detected on all ten transmissions, all ten IL2P
 *     headers decode, and nine of the ten payloads come out with the beacon
 *     text and callsigns intact.
 *   - here to NinoTNC: a frame from this firmware's transmitter fed through
 *     pymodem's IL2P codec decodes first time, "Bytes Corrected: 0".
 *
 * WHAT IS STILL NOT RIGHT
 *
 * The receiver is spending most of its forward error correction just to get
 * these frames out. Reed-Solomon corrections per payload block, on a clean,
 * noiseless, AGC free capture, where the code can carry eight:
 *
 *     3  2  8  2  1  5  1  7  -1  7
 *
 * One frame in ten goes over. pymodem needs zero corrections on a comparable
 * signal, so this is not inherent to the waveform.
 *
 * Sensitivity to a sample clock offset is the likely cause and is easy to
 * measure. Resampling the capture and counting decodes:
 *
 *     -500ppm 0/10   -250ppm 1/10   0ppm 9/10
 *     +250ppm 9/10   +500ppm 7/10   +1000ppm 0/10
 *
 * That is a usable window of roughly 500 ppm, and it is not centred on zero.
 * Mode2RX has no symbol clock tracking at all: correlateSync() fixes the
 * sampling instant once and samplesToBits() then steps a constant
 * MODE2_RADIO_SYMBOL_LENGTH for the whole frame, so any difference between
 * the transmitter's symbol clock and the receiver's ADC clock accumulates
 * across 464 payload symbols with nothing pulling it back.
 *
 * The tests below assert what the receiver does today. The nine of ten figure
 * is a statement of the current state, not a target.
  */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <string>

namespace {

  const char* CAPTURE_9600 = "../testdata/ninotnc-v44-9600-C4FSK-IL2Pc-0011.wav";

  /* The firmware's own transmitter peaks around 695 counts either side of the
     mid rail, so that is the natural level to present a capture at. */
  const int NOMINAL_PEAK = 695;

  std::vector<uint16_t> capture(int peakCounts)
  {
    unsigned rate = 0U;
    const std::vector<int16_t> pcm = radio::loadWav(CAPTURE_9600, rate);

    if (pcm.empty() || rate != 48000U)
      return std::vector<uint16_t>();

    /* Mode 2 runs at 24 kHz. */
    return radio::toAdc(radio::decimate(pcm, 2U), peakCounts);
  }

  unsigned count(const std::string& haystack, const std::string& needle)
  {
    unsigned n = 0U;
    for (size_t p = haystack.find(needle); p != std::string::npos; p = haystack.find(needle, p + 1U))
      n++;
    return n;
  }

}

TF_TEST(ninotnc_capture_loads)
{
  unsigned rate = 0U;
  const std::vector<int16_t> pcm = radio::loadWav(CAPTURE_9600, rate);

  REQUIRE_MSG(!pcm.empty(), "could not read " << CAPTURE_9600);
  CHECK_EQ(int(rate), 48000);
  CHECK_MSG(pcm.size() > 480000U, "only " << pcm.size() << " samples");
}

TF_TEST(ninotnc_9600_sync_is_detected)
{
  /*
   * The strongest statement available about interoperability: this firmware
   * locks to a NinoTNC's sync vector on a signal it had no part in
   * generating, ten times out of ten.
   */
  const std::vector<uint16_t> adc = capture(NOMINAL_PEAK);
  REQUIRE(!adc.empty());

  radio::demodulate(adc);

  const unsigned syncs = count(hooks::g_debugTx, "valid sync vector");

  CHECK_MSG(syncs >= 10U,
            "expected at least one sync report per transmission, got " << syncs);

  CHECK_MSG(hooks::debugContains("sync found"),
            "sync was never accepted; the threshold gate rejected every candidate");
}

TF_TEST(ninotnc_9600_header_decodes)
{
  /*
   * Getting the header out means the sync position, symbol timing, level
   * slicing, dibit mapping, descrambling and Reed-Solomon all agree with the
   * NinoTNC. The length it reports has to be the 100 bytes the beacon script
   * sent.
   */
  const std::vector<uint16_t> adc = capture(NOMINAL_PEAK);
  REQUIRE(!adc.empty());

  radio::demodulate(adc);

  CHECK_MSG(hooks::debugContains("IL2PRX: type 1 header"),
            "the header did not decode as an IL2P type 1 header");

  CHECK_MSG(hooks::debugContains("header is valid and has a payload 100"),
            "the header did not report a 100 byte payload");

  const unsigned headers = count(hooks::g_debugTx, "header is valid");
  CHECK_MSG(headers >= 10U,
            "expected a header from each of the 10 transmissions, got " << headers);
}

TF_TEST(ninotnc_9600_header_decodes_across_a_range_of_receive_levels)
{
  /* A real link does not arrive at a convenient level. */
  const int peaks[] = { 300, 500, 695, 900 };

  for (unsigned i = 0U; i < 4U; i++) {
    const std::vector<uint16_t> adc = capture(peaks[i]);
    REQUIRE(!adc.empty());

    radio::demodulate(adc);

    const unsigned headers = count(hooks::g_debugTx, "header is valid");

    CHECK_MSG(headers >= 10U,
              "at a peak of " << peaks[i] << " counts: " << headers << " headers of 10");
  }
}

TF_TEST(ninotnc_9600_frames_decode_end_to_end)
{
  /* The whole point: real off-air frames from an independent implementation,
     all the way out to KISS. */
  const std::vector<uint16_t> adc = capture(NOMINAL_PEAK);
  REQUIRE(!adc.empty());

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(adc);

  unsigned beacons = 0U;
  for (size_t i = 0U; i < frames.size(); i++) {
    const std::string body(frames[i].begin() + 1, frames[i].end());
    if (body.find("Beacon from M0LTE") != std::string::npos)
      beacons++;
  }

  CHECK_MSG(beacons >= 9U,
            "expected at least 9 of the 10 beacons; got " << beacons
            << " from " << frames.size() << " decode(s)");

  /* The AX.25 addresses have to survive the IL2P type 1 translation too. */
  REQUIRE(!frames.empty());
  const std::vector<uint8_t>& f = frames[0];
  REQUIRE(f.size() > 16U);
  CHECK_EQ(int(f[15]), int(0x03U));            /* UI */
  CHECK_EQ(int(f[16]), int(0xF0U));            /* no layer 3 */
}

TF_TEST(ninotnc_9600_all_ten_frames_decode)
{
  /*
   * Originally one of the ten payloads exceeded what errors-only
   * Reed-Solomon could carry and this test pinned down that the receiver
   * refused it rather than passing it to the host. The timing search and
   * erasure decoding recover that frame now, so the capture decodes in
   * full; what remains pinned is that nothing bogus is delivered alongside.
   */
  const std::vector<uint16_t> adc = capture(NOMINAL_PEAK);
  REQUIRE(!adc.empty());

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(adc);

  unsigned beacons = 0U;
  for (size_t i = 0U; i < frames.size(); i++) {
    const std::string body(frames[i].begin() + 1, frames[i].end());
    if (body.find("Beacon from M0LTE") != std::string::npos)
      beacons++;
  }

  CHECK_MSG(beacons == 10U, "expected all 10 beacons; got " << beacons);
  CHECK_MSG(frames.size() == 10U,
            "expected exactly the 10 real frames and nothing else; got " << frames.size());
}

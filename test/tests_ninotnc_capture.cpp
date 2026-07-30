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
 * WHAT INTEROPERATES AND WHAT DOES NOT
 *
 * Sync detection and the IL2P header interoperate exactly. Offline analysis of
 * the capture confirms it independently of this firmware:
 *
 *   - MODE2_SYNC_SYMBOLS_VALUES matches the transmitted sync vector with 0 of
 *     16 symbols wrong, in every burst of both captures
 *   - the 15 byte header is a valid RS(15,13) codeword, syndromes zero, and
 *     decodes to a type 1 MaxFEC header declaring a 100 byte payload
 *
 * The payload block does not. Its codeword does not validate under this
 * firmware's assumptions, and an offline search over start offset, block
 * length, 16/24/32/48 RS roots, first-consecutive-root 0 and 1, scrambler
 * ordering, and symbol rate within +/-0.4% found no combination that makes it
 * one. So the two agree on the physical layer and the header, and diverge on
 * the payload FEC. Running that down needs Nino's IL2P specification rather
 * than more guessing, so these tests assert what has been established and no
 * more.
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

TF_TEST(ninotnc_9600_undecodable_payload_is_not_passed_to_the_host)
{
  /*
   * The payload FEC does not currently interoperate, which is a real gap. The
   * thing that must not happen meanwhile is the receiver handing the host a
   * frame it could not decode.
   *
   * The Reed-Solomon decoder returns -1 for this block. Until that negative
   * return was honoured it was read as success and the corrupt payload went on
   * to the CRC. This pins the fix down on real off-air data rather than a
   * synthetic case.
   */
  const std::vector<uint16_t> adc = capture(NOMINAL_PEAK);
  REQUIRE(!adc.empty());

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(adc);

  CHECK_MSG(hooks::debugContains("payload is invalid"),
            "expected the undecodable payload to be reported as invalid");

  CHECK_MSG(frames.empty(),
            "a frame whose payload failed FEC was passed to the host: "
            << frames.size() << " frame(s)");
}

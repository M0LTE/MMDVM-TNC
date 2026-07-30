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
 * A hundred frames off a real NinoTNC.
 *
 * The ten frame capture is enough to prove the two implementations talk to
 * each other. It is not enough to measure how well. This one is 108 seconds,
 * a hundred numbered beacons, and it is the yardstick for the receiver's
 * real world reliability.
 *
 * The goal is 100 of 100. Anything less is the receiver throwing away frames
 * that a clean, noiseless recording plainly contains -- every one of these
 * transmissions produces a sync detection and a valid IL2P header, so the
 * signal is there and the loss is all in the payload block.
 *
 * The assertion below tracks where the receiver actually is, and is meant to
 * be raised as it improves. It exists to stop the number going backwards.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <cstdio>
#include <cstdlib>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>

namespace {

  const char* BULK_9600 = "../testdata/ninotnc-v44-9600-C4FSK-IL2Pc-0011-100-frames.wav";

  /* Distinct beacon numbers recovered, so a duplicate decode cannot inflate
     the count. */
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

TF_TEST(ninotnc_bulk_9600_sync_and_header_are_perfect)
{
  /*
   * Establishes that the capture is clean and that everything up to the
   * payload block works, so any shortfall below is squarely the payload.
   */
  unsigned rate = 0U;
  const std::vector<int16_t> pcm = radio::loadWav(BULK_9600, rate);
  REQUIRE_MSG(!pcm.empty(), "could not read " << BULK_9600);

  radio::demodulate(radio::toAdc(radio::decimate(pcm, 2U), 695));

  unsigned syncs = 0U, headers = 0U;
  const std::string& d = hooks::g_debugTx;
  for (size_t p = d.find("sync found"); p != std::string::npos; p = d.find("sync found", p + 1U))
    syncs++;
  for (size_t p = d.find("header is valid"); p != std::string::npos; p = d.find("header is valid", p + 1U))
    headers++;

  CHECK_MSG(syncs   >= 100U, "only " << syncs   << " of 100 transmissions produced a sync");
  CHECK_MSG(headers >= 100U, "only " << headers << " of 100 headers decoded");
}

TF_TEST(ninotnc_bulk_9600_decode_rate)
{
  unsigned rate = 0U;
  const std::vector<int16_t> pcm = radio::loadWav(BULK_9600, rate);
  REQUIRE(!pcm.empty());

  const unsigned got = beaconsDecoded(radio::demodulate(radio::toAdc(radio::decimate(pcm, 2U), 695)));

  std::printf("      decoded %u of 100 beacons\n", got);

  /* Raise this as the receiver improves. The target is 100; the two frames
     short of it have a visibly degraded eye in the recording itself and are
     under investigation against the reference decoder. */
  CHECK_MSG(got >= 98U,
            "decoded " << got << " of 100 beacons; the floor is 98 and the target is 100");
}

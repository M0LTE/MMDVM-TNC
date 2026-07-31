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
 * Sample clock error. Transmit and receive oscillators are never the same
 * frequency: two crystal-timed TNCs can easily sit 100 ppm apart, and the
 * sampling instant then slides continuously through the symbol instead of
 * standing still the way the fixed phase offsets elsewhere in the suite do.
 * This is the impairment the mode 2/3 receiver's timing grid search exists
 * for, and nothing else in the suite exercises it.
 *
 * Measured today all three modes decode cleanly at +/-1000 ppm; the tests
 * assert +/-500, five times anything real hardware will show them.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"

namespace {

  bool decodesAt(uint8_t mode, size_t length, int ppm, unsigned phase)
  {
    const std::vector<uint8_t> payload = radio::rawPayload(length);

    std::vector<uint16_t> wave = radio::modulate(&payload[0], uint16_t(payload.size()),
                                                 mode == 1U ? 30U : 3U, mode);

    radio::Channel ch;
    ch.phase = phase;

    wave = radio::applyClockError(radio::applyChannel(wave, ch), ppm);

    return radio::decodedExactly(radio::demodulate(wave, true, mode), payload);
  }

  void sweep(uint8_t mode, const char* name)
  {
    const int ppms[] = { -500, -200, -100, 100, 200, 500 };

    for (unsigned i = 0U; i < sizeof(ppms) / sizeof(ppms[0]); i++) {
      for (unsigned phase = 0U; phase < 5U; phase += 2U) {
        CHECK_MSG(decodesAt(mode, 48U, ppms[i], phase),
                  name << " at " << ppms[i] << " ppm, phase " << phase);
      }
    }
  }

}

TF_TEST(mode2_decodes_with_sample_clock_error)
{
  sweep(2U, "mode 2");
}

TF_TEST(mode3_decodes_with_sample_clock_error)
{
  sweep(3U, "mode 3");
}

TF_TEST(ax25_decodes_with_sample_clock_error)
{
  sweep(1U, "mode 1");
}

TF_TEST(mode2_longest_frame_survives_clock_error)
{
  /* Drift accumulates over the burst, so the 1023 byte frame is the hard
     case: at 150 ppm the sampling point moves several samples between the
     sync vector and the trailing CRC. */
  CHECK_MSG(decodesAt(2U, 1023U,  150, 1U), "1023 bytes at +150 ppm");
  CHECK_MSG(decodesAt(2U, 1023U, -150, 3U), "1023 bytes at -150 ppm");
}

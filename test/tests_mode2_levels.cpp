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
 * Slicing levels: receive level and residual DC.
 *
 * calculateLevels() estimates the centre of the eye and the +/-3 versus +/-1
 * decision threshold from the section it is about to slice, and samplesToBits()
 * applies them. Both have to work in either deviation sense, because the
 * receiver is supposed to decode a transmission of either polarity.
 *
 * CIO::process() only removes the nominal 2048 mid rail, and the receive
 * filter passes DC, so whatever offset the ADC actually has arrives at the
 * slicer intact. Removing it is calculateLevels()' and samplesToBits()' job.
 */

#include "Config.h"
#include "Globals.h"

#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

namespace {

  /*
   * CIO::process() halves the ADC swing on its way in, so an offset of A LSB
   * at the ADC lands as A/2 at the slicer. The nominal symbol levels there
   * are about +/-340 and +/-113, with the 3-versus-1 threshold near 227.
   */
  const int SMALL_DC = 120;
  const int LARGE_DC = 160;

  /*
   * Both are inside what the slicer can cope with by design.
   *
   * calculateLevels() partitions samples by sign to find the +3/+1 and -3/-1
   * pairs, so it stops working once the offset is large enough to put the
   * +/-1 level on the wrong side of zero. Measured, the normal sense decodes
   * from -160 to +200 LSB and fails outside that; the envelope is asymmetric
   * because a sample of exactly zero falls into the negative bucket. Nothing
   * here asks for more than the receiver can give.
   */

  bool decodesWith(bool invert, int dcOffset, double gain = 1.0)
  {
    const std::vector<uint8_t> payload = radio::rawPayload(48U);

    radio::Channel ch;
    ch.invert   = invert;
    ch.dcOffset = dcOffset;
    ch.gain     = gain;

    return radio::decodedExactly(radio::loopback(payload, ch), payload);
  }

}

TF_TEST(mode2_tolerates_dc_offset_in_the_normal_sense)
{
  CHECK_MSG(decodesWith(false, SMALL_DC), "normal deviation, " << SMALL_DC << " LSB of DC");
  CHECK_MSG(decodesWith(false, LARGE_DC), "normal deviation, " << LARGE_DC << " LSB of DC");
  CHECK_MSG(decodesWith(false, -LARGE_DC), "normal deviation, " << -LARGE_DC << " LSB of DC");
}

TF_TEST(mode2_tolerates_dc_offset_in_the_inverted_sense)
{
  /*
   * The same offsets with the deviation the other way up. Nothing about a
   * DC offset on the ADC input cares which way the transmitter deviates, so
   * these must behave identically to the test above.
   */
  CHECK_MSG(decodesWith(true, SMALL_DC), "inverted deviation, " << SMALL_DC << " LSB of DC");
  CHECK_MSG(decodesWith(true, LARGE_DC), "inverted deviation, " << LARGE_DC << " LSB of DC");
  CHECK_MSG(decodesWith(true, -LARGE_DC), "inverted deviation, " << -LARGE_DC << " LSB of DC");
}

TF_TEST(mode2_dc_offset_behaves_the_same_in_both_senses)
{
  /* Stated as a symmetry, which is the property that actually broke. */
  for (int dc = -LARGE_DC; dc <= LARGE_DC; dc += LARGE_DC) {
    const bool normal   = decodesWith(false, dc);
    const bool inverted = decodesWith(true,  dc);

    CHECK_MSG(normal == inverted,
              "with " << dc << " LSB of DC: normal deviation "
              << (normal ? "decoded" : "lost") << ", inverted deviation "
              << (inverted ? "decoded" : "lost"));
  }
}

TF_TEST(mode2_tolerates_a_range_of_receive_levels)
{
  /* Unrelated to DC, but the same slicer. A quiet or loud signal must decode
     in either sense. */
  const double gains[] = { 0.35, 0.7, 1.0 };

  for (unsigned i = 0U; i < 3U; i++) {
    CHECK_MSG(decodesWith(false, 0, gains[i]), "normal deviation at gain " << gains[i]);
    CHECK_MSG(decodesWith(true,  0, gains[i]), "inverted deviation at gain " << gains[i]);
  }
}

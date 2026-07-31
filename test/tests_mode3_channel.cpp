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
 * Channel impairments for mode 3, mirroring the sweeps mode 2 has had all
 * along: noise, receive level and DC offset. Mode 3 is generated from the
 * mode 2 sources but compiled separately at twice the sample rate, so
 * nothing about mode 2's margins is automatically true of it.
 *
 * Measured today: 100% decode to 200 LSB of peak noise, the shoulder at
 * about 250 -- the same shape as mode 2 against the same ~695 LSB peak.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"

namespace {

  bool decodesWith(const radio::Channel& ch)
  {
    const std::vector<uint8_t> payload = radio::rawPayload(48U);

    const std::vector<uint16_t> wave = radio::modulate(&payload[0], uint16_t(payload.size()), 3U, 3U);

    return radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch), true, 3U), payload);
  }

  unsigned decodeRate(unsigned noise, unsigned trials)
  {
    unsigned decoded = 0U;

    for (unsigned t = 0U; t < trials; t++) {
      radio::Channel ch;
      ch.noise  = noise;
      ch.seed   = 5000U + t * 13U;
      ch.phase  = t % 5U;
      ch.invert = (t % 2U) != 0U;

      if (decodesWith(ch))
        decoded++;
    }

    return decoded;
  }

}

TF_TEST(mode3_decodes_everything_on_a_lightly_noisy_channel)
{
  const unsigned TRIALS = 20U;

  const unsigned decoded = decodeRate(150U, TRIALS);

  CHECK_MSG(decoded == TRIALS,
            "150 LSB of peak noise cost " << (TRIALS - decoded) << " of "
            << TRIALS << " packets; it should cost none");
}

TF_TEST(mode3_still_works_on_a_thoroughly_noisy_channel)
{
  /* Measured at 100% today; asserting 90% leaves room for arithmetic
     differences between compilers while catching a real sensitivity loss. */
  const unsigned TRIALS = 20U;

  const unsigned decoded = decodeRate(200U, TRIALS);

  CHECK_MSG(decoded * 100U >= TRIALS * 90U,
            "200 LSB of peak noise decoded only " << decoded << " of " << TRIALS);
}

TF_TEST(mode3_tolerates_a_range_of_receive_levels)
{
  const double gains[] = { 0.25, 0.5, 2.0, 3.0 };

  for (unsigned i = 0U; i < sizeof(gains) / sizeof(gains[0]); i++) {
    for (unsigned phase = 0U; phase < 5U; phase += 2U) {
      radio::Channel ch;
      ch.gain  = gains[i];
      ch.phase = phase;

      CHECK_MSG(decodesWith(ch), "gain " << gains[i] << ", phase " << phase);
    }
  }
}

TF_TEST(mode3_tolerates_dc_offset_in_both_senses)
{
  const int dcs[] = { 120, 160, -160 };

  for (unsigned i = 0U; i < sizeof(dcs) / sizeof(dcs[0]); i++) {
    for (unsigned inv = 0U; inv < 2U; inv++) {
      radio::Channel ch;
      ch.dcOffset = dcs[i];
      ch.invert   = inv != 0U;

      CHECK_MSG(decodesWith(ch), (inv ? "inverted" : "normal") << " sense, " << dcs[i] << " LSB of DC");
    }
  }
}

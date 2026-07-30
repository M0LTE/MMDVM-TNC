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
 * The receive filter against the CMSIS-DSP contract.
 *
 * arm_fir_init_q15() documents it plainly: "numTaps must be even and greater
 * than or equal to 4", returning ARM_MATH_ARGUMENT_ERROR otherwise. The
 * firmware never calls the init function -- it fills the
 * arm_fir_instance_q15 struct by hand -- so that check never runs and an
 * illegal tap count reaches the kernel unchallenged.
 *
 * arm_fir_fast_q15() relies on the same invariant. Its tap loop is unrolled
 * four ways and the remainder branch is guarded only by
 * `if ((numTaps & 0x3U) != 0U)`, with the CMSIS comment:
 *
 *   "If the filter length is not a multiple of 4, compute the remaining
 *    filter taps. This is always be 2 taps since the filter length is even."
 *
 * With an odd numTaps that branch still runs and still consumes two taps, so
 * the kernel reads one coefficient past the end of the array and one sample
 * past the end of the state block. On target that extra coefficient is
 * whatever .data happens to sit after RX_FILTER.
 *
 * The shim reproduces that arithmetic and counts it, reading zero rather than
 * actually indexing out of bounds so the harness stays memory safe.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

TF_TEST(rx_filter_uses_a_legal_tap_count)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> burst  = radio::modulate(&payload[0], uint16_t(payload.size()));

  armshim::resetCounters();

  radio::demodulate(burst);

  CHECK_MSG(armshim::g_oddTapCalls == 0UL,
            "arm_fir_fast_q15 was called " << armshim::g_oddTapCalls
            << " times with a tap count that is odd or below 4");

  CHECK_MSG(armshim::g_coeffOverrunReads == 0UL,
            "the filter kernel read past the end of its coefficient array "
            << armshim::g_coeffOverrunReads << " times");
}

TF_TEST(rx_filter_still_passes_traffic)
{
  /* The tap count fix must not change what the filter does. */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  for (unsigned phase = 0U; phase < 5U; phase++) {
    radio::Channel ch;
    ch.phase = phase;

    CHECK_MSG(radio::decodedExactly(radio::loopback(payload, ch), payload),
              "lost the packet at sampling phase " << phase);
  }
}

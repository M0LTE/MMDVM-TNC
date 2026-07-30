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

#include <vector>

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

TF_TEST(rx_filter_leading_zero_computes_the_intended_convolution)
{
  /*
   * The fix pads the designed 45 taps to an even 46 with a zero on the front.
   * "Identical to before" is not a claim worth making, because before was not
   * well defined -- CMSIS was reading a 46th coefficient from whatever
   * followed the array. What can be shown is that the padded filter computes
   * exactly the convolution the 45 designed taps describe.
   *
   * CMSIS runs coefficients against the state buffer oldest first:
   *
   *   y[n] = sum over j in [0, N) of c[j] * x[n - N + 1 + j]
   *
   * so with c = { 0, d0 .. d4 } and N = 6 that collapses to
   *
   *   y[n] = sum over k in [0, 5) of d[k] * x[n - 4 + k]
   *
   * which is the five tap design, no delay added.
   */
  const q15_t designed[5] = { 700, -2400, 9000, -2400, 700 };
  q15_t       padded[6]   = { 0, 700, -2400, 9000, -2400, 700 };

  q15_t state[16] = { 0 };

  arm_fir_instance_q15 f;
  f.numTaps = 6U;
  f.pState  = state;
  f.pCoeffs = padded;

  std::vector<q15_t> history;
  for (unsigned i = 0U; i < 4U; i++)
    history.push_back(0);                 /* x[m] = 0 for m < 0 */

  /* Two samples at a time, the block size the receiver actually uses. */
  for (int block = 0; block < 32; block++) {
    q15_t in[2];
    in[0] = q15_t(((block * 37) % 19) * 700 - 6000);
    in[1] = q15_t(((block * 11) % 23) * 500 - 5000);

    q15_t out[2] = { 0, 0 };
    ::arm_fir_fast_q15(&f, in, out, 2U);

    for (unsigned s = 0U; s < 2U; s++) {
      history.push_back(in[s]);

      q31_t acc = 0;
      const size_t n = history.size() - 1U;
      for (unsigned k = 0U; k < 5U; k++)
        acc += q31_t(history[n - 4U + k]) * q31_t(designed[k]);

      const q15_t expected = q15_t(__SSAT(acc >> 15, 16));

      CHECK_MSG(out[s] == expected,
                "block " << block << " sample " << s << ": filter gave "
                << int(out[s]) << ", the five tap design gives " << int(expected));
    }
  }

  CHECK_EQ(int(armshim::g_oddTapCalls), 0);
  CHECK_EQ(int(armshim::g_coeffOverrunReads), 0);
}

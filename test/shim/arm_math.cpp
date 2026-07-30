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

#include <arm_math.h>

#include <cstring>

namespace armshim {
  unsigned long g_oddTapCalls       = 0UL;
  unsigned long g_coeffOverrunReads = 0UL;

  void resetCounters()
  {
    g_oddTapCalls       = 0UL;
    g_coeffOverrunReads = 0UL;
  }
}

/*
 * Q15 FIR, CMSIS state layout.
 *
 * pState holds (numTaps - 1) samples of history followed by room for
 * blockSize new samples. Output n is computed from the oldest sample forward
 * against pCoeffs forward, which is the time reversed convention CMSIS
 * documents.
 */
extern "C" void arm_fir_fast_q15(const arm_fir_instance_q15* S, q15_t* pSrc, q15_t* pDst, uint32_t blockSize)
{
  const uint16_t numTaps = S->numTaps;
  q15_t*         pState  = S->pState;
  const q15_t*   pCoeffs = S->pCoeffs;

  /* The documented precondition, which the real CMSIS kernel assumes without
     checking. See the long comment in arm_math.h. */
  const bool legal = (numTaps >= 4U) && ((numTaps & 1U) == 0U);
  if (!legal)
    armshim::g_oddTapCalls++;

  /* How many taps the real kernel actually touches: four at a time, plus an
     unconditional pair whenever numTaps is not a multiple of four. */
  const uint32_t effTaps = ((numTaps & 3U) != 0U) ? (uint32_t(numTaps >> 2) * 4U + 2U)
                                                  : uint32_t(numTaps);

  q15_t* pStateCurnt = pState + (numTaps - 1U);

  for (uint32_t n = 0U; n < blockSize; n++) {
    pStateCurnt[n] = pSrc[n];

    q31_t acc = 0;

    for (uint32_t j = 0U; j < effTaps; j++) {
      /* Past the declared end of the arrays the real kernel reads adjacent
         memory. Count it and substitute zero. */
      if (j >= numTaps) {
        armshim::g_coeffOverrunReads++;
        continue;
      }

      acc += q31_t(pState[n + j]) * q31_t(pCoeffs[j]);
    }

    pDst[n] = q15_t(__SSAT(acc >> 15, 16));
  }

  /* Retain the trailing (numTaps - 1) samples for the next block. */
  if (numTaps > 1U)
    ::memmove(pState, pState + blockSize, (numTaps - 1U) * sizeof(q15_t));
}

extern "C" void arm_fir_f32(const arm_fir_instance_f32* S, float32_t* pSrc, float32_t* pDst, uint32_t blockSize)
{
  const uint16_t   numTaps = S->numTaps;
  float32_t*       pState  = S->pState;
  const float32_t* pCoeffs = S->pCoeffs;

  float32_t* pStateCurnt = pState + (numTaps - 1U);

  for (uint32_t n = 0U; n < blockSize; n++) {
    pStateCurnt[n] = pSrc[n];

    float32_t acc = 0.0F;
    for (uint32_t j = 0U; j < numTaps; j++)
      acc += pState[n + j] * pCoeffs[j];

    pDst[n] = acc;
  }

  if (numTaps > 1U)
    ::memmove(pState, pState + blockSize, (numTaps - 1U) * sizeof(float32_t));
}

/*
 * Q15 polyphase interpolator, CMSIS semantics.
 *
 * pState holds (phaseLength - 1) samples of history followed by room for
 * blockSize new samples. For each input sample L outputs are produced, output
 * phase p using coefficients pCoeffs[p], pCoeffs[p + L], pCoeffs[p + 2L] ...
 * against the state buffer running oldest first.
 */
extern "C" void arm_fir_interpolate_q15(const arm_fir_interpolate_instance_q15* S, q15_t* pSrc, q15_t* pDst, uint32_t blockSize)
{
  const uint8_t  L           = S->L;
  const uint16_t phaseLength = S->phaseLength;
  const q15_t*   pCoeffs     = S->pCoeffs;
  q15_t*         pState      = S->pState;

  q15_t* pStateCurnt = pState + (phaseLength - 1U);

  uint32_t out = 0U;

  for (uint32_t n = 0U; n < blockSize; n++) {
    pStateCurnt[n] = pSrc[n];

    for (uint8_t p = 0U; p < L; p++) {
      q63_t acc = 0;

      for (uint32_t j = 0U; j < phaseLength; j++)
        acc += q63_t(pState[n + j]) * q63_t(pCoeffs[j * L + p]);

      pDst[out++] = q15_t(__SSAT(int32_t(acc >> 15), 16));
    }
  }

  if (phaseLength > 1U)
    ::memmove(pState, pState + blockSize, (phaseLength - 1U) * sizeof(q15_t));
}

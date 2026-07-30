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
 * Host stand-in for the subset of CMSIS-DSP that this firmware uses.
 *
 * The whole CMSIS surface used by MMDVM-TNC is:
 *
 *   arm_fir_fast_q15 / arm_fir_instance_q15
 *   arm_fir_f32 / arm_fir_instance_f32
 *   arm_fir_interpolate_q15 / arm_fir_interpolate_instance_q15
 *   __SSAT
 *   float32_t
 *
 * The struct layouts and the coefficient ordering match the real CMSIS-DSP
 * exactly, including the fact that pCoeffs is stored in TIME REVERSED order:
 *
 *   "pCoeffs points to the array of filter coefficients stored in time
 *    reversed order: {b[numTaps-1], b[numTaps-2], ..., b[1], b[0]}"
 *
 * (All of the filters in this firmware happen to be symmetric, so the
 * reversal is not observable, but the shim follows CMSIS regardless.)
 */

#if !defined(ARM_MATH_H_SHIM)
#define  ARM_MATH_H_SHIM

#include <cstdint>
#include <cstddef>

typedef int8_t   q7_t;
typedef int16_t  q15_t;
typedef int32_t  q31_t;
typedef int64_t  q63_t;
typedef float    float32_t;
typedef double   float64_t;

/* Saturate `val` to a signed `bits`-wide range, as per the Cortex-M SSAT
   instruction. Only the 16 bit form is used by this firmware but the general
   case is cheap to provide. */
static inline int32_t __SSAT(int32_t val, uint32_t bits)
{
  const int32_t max =  (int32_t(1) << (bits - 1)) - 1;
  const int32_t min = -(int32_t(1) << (bits - 1));

  if (val > max)
    return max;
  if (val < min)
    return min;

  return val;
}

typedef struct {
  uint16_t       numTaps;
  q15_t*         pState;
  const q15_t*   pCoeffs;
} arm_fir_instance_q15;

typedef struct {
  uint16_t       numTaps;
  float32_t*     pState;
  const float32_t* pCoeffs;
} arm_fir_instance_f32;

typedef struct {
  uint8_t        L;
  uint16_t       phaseLength;
  const q15_t*   pCoeffs;
  q15_t*         pState;
} arm_fir_interpolate_instance_q15;

extern "C" {

void arm_fir_fast_q15(const arm_fir_instance_q15* S, q15_t* pSrc, q15_t* pDst, uint32_t blockSize);
void arm_fir_f32(const arm_fir_instance_f32* S, float32_t* pSrc, float32_t* pDst, uint32_t blockSize);
void arm_fir_interpolate_q15(const arm_fir_interpolate_instance_q15* S, q15_t* pSrc, q15_t* pDst, uint32_t blockSize);

}

/*
 * Test observability for the documented arm_fir_fast_q15 precondition.
 *
 * arm_fir_init_q15() rejects an odd tap count outright:
 *
 *   "numTaps must be even and greater than or equal to 4."
 *   Returns ARM_MATH_ARGUMENT_ERROR "if numTaps is not greater than or equal
 *   to 4 and even".
 *
 * MMDVM-TNC never calls arm_fir_init_q15 -- it populates the
 * arm_fir_instance_q15 struct by hand -- so that argument check never runs and
 * an illegal tap count reaches the filter kernel silently.
 *
 * arm_fir_fast_q15() itself relies on the same invariant. Its tap loop is
 * unrolled four ways and the remainder branch is guarded only by
 * `if ((numTaps & 0x3U) != 0U)`, with the comment:
 *
 *   "If the filter length is not a multiple of 4, compute the remaining filter
 *    taps. This is always be 2 taps since the filter length is even."
 *
 * With an odd numTaps that branch still runs and still consumes two taps, so
 * the kernel reads (numTaps | 1) + 1 coefficients -- one past the end of the
 * coefficient array, and one past the end of the state block.
 *
 * The shim reproduces that arithmetic so the overrun is observable, but reads
 * zero rather than actually indexing out of bounds, so the harness itself
 * stays memory safe and the remaining tests are not perturbed by whatever
 * happens to sit after the array.
 */
namespace armshim {
  extern unsigned long g_oddTapCalls;        // calls made with an illegal numTaps
  extern unsigned long g_coeffOverrunReads;  // coefficient reads past numTaps-1
  void resetCounters();
}

#endif

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
 * Fuzz the mode 2 receiver at the sample level: input bytes become the q15
 * sample stream the RRC filter, sync correlator, timing grid search and
 * slicers all run on. On the air this stream is whatever the ADC picked up,
 * so it can be anything at all; a seed corpus of real bursts gives the
 * fuzzer sync vectors and plausible frames to mutate.
 *
 * Every pair of input bytes is one little-endian q15 sample.
 */

#include "Config.h"
#include "Globals.h"

#include "shim/TestHooks.h"

#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  hooks::reset();

  m_mode = 2U;
  mode2RX.reset();

  q15_t block[RX_BLOCK_SIZE];

  size_t i = 0U;
  while (i + 2U * RX_BLOCK_SIZE <= size) {
    for (uint16_t j = 0U; j < RX_BLOCK_SIZE; j++, i += 2U)
      block[j] = q15_t(int16_t(uint16_t(data[i]) | (uint16_t(data[i + 1U]) << 8)));

    mode2RX.samples(block, RX_BLOCK_SIZE);
  }

  return 0;
}

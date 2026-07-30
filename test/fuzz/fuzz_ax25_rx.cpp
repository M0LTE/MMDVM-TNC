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
 * Fuzz the mode 1 receiver at the sample level: the bandpass filter, all
 * three AFSK demodulators with their different equalisers, the HDLC
 * deframer and the frame CRC check. Every pair of input bytes is one
 * little-endian q15 sample.
 *
 * The receiver is constructed fresh for every input, so each run is
 * reproducible from its input alone.
 */

#include "Config.h"
#include "Globals.h"

#include "AX25RX.h"

#include "shim/TestHooks.h"

#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  hooks::reset();

  m_mode = 1U;

  CAX25RX rx;

  q15_t block[RX_BLOCK_SIZE];

  size_t i = 0U;
  while (i + 2U * RX_BLOCK_SIZE <= size) {
    for (uint16_t j = 0U; j < RX_BLOCK_SIZE; j++, i += 2U)
      block[j] = q15_t(int16_t(uint16_t(data[i]) | (uint16_t(data[i + 1U]) << 8)));

    rx.samples(block, RX_BLOCK_SIZE);
  }

  return 0;
}

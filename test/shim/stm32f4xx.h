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
 * Host stand-in for the ST peripheral library header.
 *
 * The firmware includes <stm32f4xx.h> from Globals.h, Utils.h and RingBuffer.h
 * purely to pull in the fixed width integer types and the C library. None of
 * the register definitions are needed by the portable sources that the test
 * harness compiles (IOSTM.cpp, SerialSTM.cpp and STMUART.cpp are excluded and
 * replaced by test/shim/Stubs.cpp), so this header only has to provide what
 * those sources rely on transitively.
 */

#if !defined(STM32F4XX_H_SHIM)
#define  STM32F4XX_H_SHIM

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>

#endif

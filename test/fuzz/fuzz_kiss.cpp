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
 * Fuzz the KISS host interface: arbitrary serial bytes through the real
 * CSerialPort parser, into processMessage()'s dispatch and on into whichever
 * transmitter the fuzzer's own SET_HARDWARE commands select. This is every
 * byte a hostile or broken host program could ever send the TNC.
 *
 * The parser is constructed fresh for every input so each run is
 * reproducible. The transmitters are the firmware's singletons and keep
 * their state between runs; only memory safety is being asserted here, not
 * their behaviour, which tests_kiss_host.cpp pins down.
 */

#include "Config.h"
#include "Globals.h"
#include "SerialPort.h"

#include "shim/TestHooks.h"

#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  hooks::reset();

  m_mode   = 2U;
  m_duplex = false;
  m_tx     = false;

  CSerialPort parser;

  hooks::g_kissRx.assign(data, data + size);
  hooks::g_kissRxPtr = 0U;

  parser.process();

  /* Run the station briefly so anything the input queued starts to go out. */
  for (unsigned t = 0U; t < 512U; t++) {
    io.interrupt();
    io.process();

    switch (m_mode) {
      case 1U:
        ax25TX.process();
        break;
      case 3U:
        mode3TX.process();
        break;
      default:
        mode2TX.process();
        break;
    }
  }

  return 0;
}

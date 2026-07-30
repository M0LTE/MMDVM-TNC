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
 * Capture points for the hardware side of CIO and CSerialPort.
 *
 * The firmware talks to the outside world through a handful of *Int() methods
 * that live in IOSTM.cpp / SerialSTM.cpp. The harness excludes those two files
 * and supplies its own in Stubs.cpp, which routes everything through here.
 */

#if !defined(TESTHOOKS_H)
#define  TESTHOOKS_H

#include <arm_math.h>

#include <string>
#include <vector>

namespace hooks {

  /* Every byte the firmware has written to serial port 1, the KISS host port. */
  extern std::vector<uint8_t> g_kissTx;

  /* Every byte the firmware has written to serial port 3, the debug port. */
  extern std::string g_debugTx;

  /* Bytes queued for the firmware to read back on port 1. */
  extern std::vector<uint8_t> g_kissRx;
  extern size_t               g_kissRxPtr;

  /* One entry per CIO::interrupt() call: the value handed to the DAC. */
  extern std::vector<uint16_t> g_dacOut;

  /* Samples CIO::interrupt() will hand to the ADC, oldest first. Once
     exhausted the ADC reads mid rail, i.e. silence. */
  extern std::vector<uint16_t> g_adcIn;
  extern size_t                g_adcInPtr;

  /* Number of CIO::interrupt() calls, i.e. elapsed 24 kHz sample ticks. */
  extern unsigned long g_ticks;

  /* PTT / DCD line state, as driven by setPTTInt() and setCOSInt(). */
  extern bool g_ptt;
  extern bool g_dcd;

  void reset();

  /* Pull complete KISS frames out of g_kissTx, unescaping as we go. Each
     returned frame still carries its leading type/address byte. */
  std::vector<std::vector<uint8_t> > kissFrames();

  /* True if the debug port has emitted the given substring. */
  bool debugContains(const char* needle);

}

#endif

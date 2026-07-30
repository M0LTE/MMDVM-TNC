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
 * Host implementations of the hardware specific halves of CIO and CSerialPort.
 *
 * These are the methods that live in IOSTM.cpp, SerialSTM.cpp and STMUART.cpp
 * on target. Everything else -- IO.cpp, SerialPort.cpp, Mode2RX.cpp,
 * Mode2TX.cpp, the IL2P and AX.25 code -- is the real firmware source,
 * compiled unmodified.
 */

#include "Config.h"
#include "Globals.h"
#include "IO.h"
#include "SerialPort.h"

#include "TestHooks.h"

namespace hooks {

  std::vector<uint8_t>  g_kissTx;
  std::string           g_debugTx;
  std::vector<uint8_t>  g_kissRx;
  size_t                g_kissRxPtr = 0U;
  std::vector<uint16_t> g_dacOut;
  std::vector<uint16_t> g_adcIn;
  size_t                g_adcInPtr  = 0U;
  unsigned long         g_ticks     = 0UL;
  bool                  g_ptt       = false;
  bool                  g_dcd       = false;

  void reset()
  {
    g_kissTx.clear();
    g_debugTx.clear();
    g_kissRx.clear();
    g_kissRxPtr = 0U;
    g_dacOut.clear();
    g_adcIn.clear();
    g_adcInPtr  = 0U;
    g_ticks     = 0UL;
    g_ptt       = false;
    g_dcd       = false;
  }

  std::vector<std::vector<uint8_t> > kissFrames()
  {
    const uint8_t KISS_FEND  = 0xC0U;
    const uint8_t KISS_FESC  = 0xDBU;
    const uint8_t KISS_TFEND = 0xDCU;
    const uint8_t KISS_TFESC = 0xDDU;

    std::vector<std::vector<uint8_t> > out;
    std::vector<uint8_t> cur;
    bool inFrame  = false;
    bool escaped  = false;

    for (size_t i = 0U; i < g_kissTx.size(); i++) {
      const uint8_t c = g_kissTx[i];

      if (c == KISS_FEND) {
        if (inFrame && !cur.empty())
          out.push_back(cur);
        cur.clear();
        inFrame = true;
        escaped = false;
        continue;
      }

      if (!inFrame)
        continue;

      if (escaped) {
        cur.push_back(c == KISS_TFEND ? KISS_FEND : (c == KISS_TFESC ? KISS_FESC : c));
        escaped = false;
      } else if (c == KISS_FESC) {
        escaped = true;
      } else {
        cur.push_back(c);
      }
    }

    return out;
  }

  bool debugContains(const char* needle)
  {
    return g_debugTx.find(needle) != std::string::npos;
  }

}

/* ---------------------------------------------------------------- CIO ---- */

static const uint16_t STUB_DC_OFFSET = 2048U;

void CIO::initInt()
{
}

void CIO::startInt()
{
}

/*
 * One 24 kHz sample tick. This mirrors the body of CIO::interrupt() in
 * IOSTM.cpp: pop a sample for the DAC, read the ADC, push it to the RX ring
 * buffer, bump the LED counter.
 */
void CIO::interrupt()
{
  uint16_t sample = STUB_DC_OFFSET;
  m_txBuffer.get(sample);
  hooks::g_dacOut.push_back(sample);

  uint16_t in = STUB_DC_OFFSET;
  if (hooks::g_adcInPtr < hooks::g_adcIn.size())
    in = hooks::g_adcIn[hooks::g_adcInPtr++];

  m_rxBuffer.put(in);

  m_ledCount++;
  hooks::g_ticks++;
}

void CIO::setLEDInt(bool on)
{
}

void CIO::setPTTInt(bool on)
{
  hooks::g_ptt = on;
}

void CIO::setCOSInt(bool on)
{
  hooks::g_dcd = on;
}

void CIO::setMode1Int(bool on)
{
}

void CIO::setMode2Int(bool on)
{
}

void CIO::setMode3Int(bool on)
{
}

void CIO::setMode4Int(bool on)
{
}

void CIO::delayInt(unsigned int dly)
{
}

uint8_t CIO::getCPU() const
{
  return 2U;
}

void CIO::getUDID(uint8_t* buffer)
{
  ::memset(buffer, 0x00U, 12U);
}

/* --------------------------------------------------------- CSerialPort ---- */

void CSerialPort::beginInt(uint8_t n, int speed)
{
}

int CSerialPort::availableForReadInt(uint8_t n)
{
  if (n != 1U)
    return 0;

  return hooks::g_kissRxPtr < hooks::g_kissRx.size() ? 1 : 0;
}

int CSerialPort::availableForWriteInt(uint8_t n)
{
  return 1;
}

uint8_t CSerialPort::readInt(uint8_t n)
{
  if (n != 1U || hooks::g_kissRxPtr >= hooks::g_kissRx.size())
    return 0U;

  return hooks::g_kissRx[hooks::g_kissRxPtr++];
}

void CSerialPort::writeInt(uint8_t n, const uint8_t* data, uint16_t length, bool flush)
{
  switch (n) {
    case 1U:
      for (uint16_t i = 0U; i < length; i++)
        hooks::g_kissTx.push_back(data[i]);
      break;
    case 3U:
      hooks::g_debugTx.append(reinterpret_cast<const char*>(data), length);
      break;
    default:
      break;
  }
}

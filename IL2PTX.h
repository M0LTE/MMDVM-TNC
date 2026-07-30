/*
 *   Copyright (C) 2023 by Jonathan Naylor G4KLX
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
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(IL2PTX_H)
#define	IL2PTX_H

#include "IL2PRS.h"
#include "AX25CRC.h"
#include "Hamming.h"

#include <cstdint>

// The header's payload byte count is a ten bit field; anything longer cannot
// be labelled and would go on the air with a wrapped count.
const uint16_t IL2P_MAX_PAYLOAD_LENGTH = 1023U;

// What the longest payload encodes to: a 15 byte protected header, the
// payload, sixteen parity symbols for each of its five blocks, and the four
// byte trailing CRC. 15 + 1023 + 80 + 4.
const uint16_t IL2P_MAX_ENCODED_LENGTH = 1122U;

class CIL2PTX {
public:
  CIL2PTX();

  uint16_t process(const uint8_t* in, uint16_t inLength, uint8_t* out);

private:
  CIL2PRS  m_rs2;
  CIL2PRS  m_rs4;
  CIL2PRS  m_rs6;
  CIL2PRS  m_rs8;
  CIL2PRS  m_rs16;
  CAX25CRC m_crc;
  CHamming m_hamming;
  uint16_t m_payloadByteCount;
  uint16_t m_payloadOffset;
  uint8_t  m_payloadBlockCount;
  uint8_t  m_smallBlockSize;
  uint8_t  m_largeBlockSize;
  uint8_t  m_largeBlockCount;
  uint8_t  m_smallBlockCount;
  uint8_t  m_paritySymbolsPerBlock;

  bool isIL2PType1(const uint8_t* frame, uint16_t length) const;
  void processType0Header(const uint8_t* in, uint16_t length, uint8_t* out);
  void processType1Header(const uint8_t* in, uint16_t length, uint8_t* out);

  void calculatePayloadBlockSize();

  void scramble(uint8_t* buffer, uint16_t length) const;

  uint8_t encode(uint8_t* buffer, uint16_t length, uint8_t numSymbols) const;
};

#endif


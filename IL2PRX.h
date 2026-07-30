/*
 *   Copyright (C) 2023,2024 by Jonathan Naylor G4KLX
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

#if !defined(IL2PRX_H)
#define	IL2PRX_H

#include "IL2PRS.h"
#include "AX25CRC.h"
#include "Hamming.h"

#include <cstdint>

class CIL2PRX {
public:
  CIL2PRX();

  bool processHeader(const uint8_t* in, uint8_t* out);
  bool processPayload(const uint8_t* in, uint8_t* out);

  // As processPayload, but marking the least confident maxErasures bytes of
  // each block as Reed-Solomon erasures. margins holds one slicing confidence
  // per input byte, smaller meaning less confident.
  bool processPayloadErasures(const uint8_t* in, uint8_t* out, const uint16_t* margins, uint8_t maxErasures);

  // Put the output cursor back to just after the header, so processPayload
  // can be attempted again with a different slicing of the same block.
  void rewindPayload();

  // Reed-Solomon corrections consumed by the most recent decode. Zero means
  // the block was a valid codeword as sliced; with only two parity symbols on
  // the header that distinction is the difference between certainty and a
  // coin toss, so callers get to see it.
  int getLastCorrections() const;

  uint16_t getHeaderLength() const;
  uint16_t getPayloadLength() const;

  uint16_t getPayloadParityLength() const;

  bool checkCRC(const uint8_t* frame, const uint8_t* crc) const;

  // The four Hamming coded CRC bytes the transmitter would have sent for
  // this frame, for soft comparison against what actually arrived.
  void expectedCRC(const uint8_t* frame, uint8_t* crc) const;

private:
  CIL2PRS  m_rs2;
  CIL2PRS  m_rs4;
  CIL2PRS  m_rs6;
  CIL2PRS  m_rs8;
  CIL2PRS  m_rs16;
  CAX25CRC m_crc;
  CHamming m_hamming;
  uint16_t m_headerByteCount;
  uint16_t m_payloadByteCount;
  uint16_t m_payloadBlockCount;
  uint16_t m_smallBlockSize;
  uint16_t m_largeBlockSize;
  uint16_t m_largeBlockCount;
  uint16_t m_smallBlockCount;
  uint16_t m_paritySymbolsPerBlock;
  uint16_t m_outOffset;
  mutable int m_lastCorrections;

  void calculatePayloadBlockSize();

  void processType0Header(const uint8_t* in, uint8_t* out);
  void processType1Header(const uint8_t* in, uint8_t* out);

  void unscramble(uint8_t* buffer, uint16_t length) const;

  bool decode(uint8_t* buffer, uint16_t length, uint8_t numSymbols) const;
  bool decodeErasures(uint8_t* buffer, uint16_t length, uint8_t numSymbols, const uint16_t* margins, uint8_t maxErasures) const;
};

#endif


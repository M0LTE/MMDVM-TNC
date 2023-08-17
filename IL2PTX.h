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

#include "Config.h"

#if !defined(IL2PTX_H)
#define  IL2PTX_H

#include <vector>

#include "IL2PTXFrame.h"

class CIL2PTX {
public:
  CIL2PTX();

  uint8_t writeData(const uint8_t* data, uint16_t length);
  uint8_t writeDataAck(uint16_t token, const uint8_t* data, uint16_t length);

  void process();

  uint8_t getSpace() const;

private:
  CIL2PTXFrame m_frame;
  uint8_t      m_poBuffer[600U];
  uint16_t     m_poLen;
  uint16_t     m_poPtr;
  uint16_t     m_tablePtr;
  std::vector<uint16_t> m_tokens;

  void writeBit(bool b);
};

#endif

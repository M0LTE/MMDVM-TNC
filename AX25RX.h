/*
 *   Copyright (C) 2020,2023 by Jonathan Naylor G4KLX
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

#if !defined(AX25RX_H)
#define  AX25RX_H

#include "AX25Demodulator.h"

class CAX25RX {
public:
  CAX25RX();

  void samples(q15_t* samples, uint8_t length);

private:
  arm_fir_instance_q15 m_filter;
  q15_t                m_state[130U];    // NoTaps + BlockSize - 1, 100 + 20 - 1 plus some spare
  CAX25Demodulator     m_demod1;
  CAX25Demodulator     m_demod2;
  CAX25Demodulator     m_demod3;
  uint16_t             m_lastFCS;
  uint32_t             m_count;
};

#endif


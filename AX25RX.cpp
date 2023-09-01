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
#include "KISSDefines.h"
#include "Globals.h"
#include "AX25RX.h"

/*
 * Generated with Scipy Filter, 152 coefficients, 1100-2300Hz bandpass,
 * Hann window, starting and ending 0 value coefficients removed.
 *
 * np.array(
 *  firwin2(152,
 *      [
 *          0.0,
 *          1000.0/(sample_rate/2),
 *          1100.0/(sample_rate/2),
 *          2350.0/(sample_rate/2),
 *          2500.0/(sample_rate/2),
 *          1.0
 *      ],
 *      [0,0,1,1,0,0],
 *      antisymmetric = False,
 *      window='hann') * 32768,
 *  dtype=int)[10:-10]
 */

const uint32_t FILTER_LEN = 130U;

q15_t FILTER_COEFFS[] = {
      5,    12,    18,    21,   19,   11,    -2,   -15,   -25,   -27,
    -21,   -11,    -3,    -5,  -19,  -43,   -69,   -83,   -73,   -35,
     27,    98,   155,   180,  163,  109,    39,   -20,   -45,   -26,
     23,    74,    89,    39,  -81, -247,  -407,  -501,  -480,  -334,
    -92,   175,   388,   479,  429,  275,    99,     5,    68,   298,
    626,   913,   994,   740,  115, -791, -1770, -2544, -2847, -2509,
  -1527,   -76,  1518,  2875, 3653, 3653,  2875,  1518,   -76, -1527,
  -2509, -2847, -2544, -1770, -791,  115,   740,   994,   913,   626,
    298,    68,     5,    99,  275,  429,   479,   388,   175,   -92,
   -334,  -480,  -501,  -407, -247,  -81,    39,    89,    74,    23,
    -26,   -45,   -20,    39,  109,  163,   180,   155,    98,    27,
    -35,   -73,   -83,   -69,  -43,  -19,    -5,    -3,   -11,   -21,
    -27,   -25,   -15,    -2,   11,   19,    21,    18,    12,     5
};

CAX25RX::CAX25RX() :
m_filter(),
m_state(),
m_demod1(3),
m_demod2(6),
m_demod3(9),
m_lastFCS(0U),
m_count(0U)
{
  m_filter.numTaps = FILTER_LEN;
  m_filter.pState  = m_state;
  m_filter.pCoeffs = FILTER_COEFFS;
}

void CAX25RX::samples(q15_t* samples, uint8_t length)
{
  q15_t output[RX_BLOCK_SIZE];
  ::arm_fir_fast_q15(&m_filter, samples, output, RX_BLOCK_SIZE);

  m_count++;

  CAX25Frame frame;

  bool ret = m_demod1.process(output, length, frame);
  if (ret) {
    if (frame.m_fcs != m_lastFCS || m_count > 2U) {
      m_lastFCS = frame.m_fcs;
      m_count   = 0U;
      serial.writeKISSData(KISS_TYPE_DATA, frame.m_data, frame.m_length - 2U);
    }
    DEBUG1("AX.25 decoder 1 reported");
  }

  ret = m_demod2.process(output, length, frame);
  if (ret) {
    if (frame.m_fcs != m_lastFCS || m_count > 2U) {
      m_lastFCS = frame.m_fcs;
      m_count   = 0U;
      serial.writeKISSData(KISS_TYPE_DATA, frame.m_data, frame.m_length - 2U);
    }
    DEBUG1("AX.25 decoder 2 reported");
  }

  ret = m_demod3.process(output, length, frame);
  if (ret) {
    if (frame.m_fcs != m_lastFCS || m_count > 2U) {
      m_lastFCS = frame.m_fcs;
      m_count   = 0U;
      serial.writeKISSData(KISS_TYPE_DATA, frame.m_data, frame.m_length - 2U);
    }
    DEBUG1("AX.25 decoder 3 reported");
  }

  bool dcd1 = m_demod1.isDCD();
  bool dcd2 = m_demod2.isDCD();
  bool dcd3 = m_demod3.isDCD();
    
  if (dcd1 || dcd2 || dcd3)
    io.setDecode(true);
  else
    io.setDecode(false);
}


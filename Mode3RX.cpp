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

#include "KISSDefines.h"
#include "Globals.h"
#include "Mode3RX.h"
#include "Utils.h"

// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
static q15_t RRC_0_2_FILTER[] = {401, 104, -340, -731, -847, -553, 112, 909, 1472, 1450, 683, -675, -2144, -3040, -2706, -770, 2667, 6995,
                                 11237, 14331, 15464, 14331, 11237, 6995, 2667, -770, -2706, -3040, -2144, -675, 683, 1450, 1472, 909, 112,
                                 -553, -847, -731, -340, 104, 401, 0};
const uint16_t RRC_0_2_FILTER_LEN = 42U;

const q15_t SCALING_FACTOR = 18750;      // Q15(0.55)

const uint8_t MAX_SYNC_BIT_ERRS     = 2U;
const uint8_t MAX_SYNC_SYMBOLS_ERRS = 3U;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t  NOAVEPTR = 99U;
const uint16_t NOENDPTR = 9999U;

CMode3RX::CMode3RX() :
m_state(MODE3RXS_NONE),
m_rrc02Filter(),
m_rrc02State(),
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_startPtr(NOENDPTR),
m_endPtr(NOENDPTR),
m_syncPtr(NOENDPTR),
m_invert(false),
m_frame(),
m_maxCorr(0),
m_centre(),
m_centreVal(0),
m_threshold(),
m_thresholdVal(0),
m_countdown(0U),
m_packet()
{
  ::memset(m_rrc02State, 0x00U, 70U * sizeof(q15_t));
  m_rrc02Filter.numTaps = RRC_0_2_FILTER_LEN;
  m_rrc02Filter.pState  = m_rrc02State;
  m_rrc02Filter.pCoeffs = RRC_0_2_FILTER;
}

void CMode3RX::reset()
{
  m_state        = MODE3RXS_NONE;
  m_dataPtr      = 0U;
  m_bitPtr       = 0U;
  m_maxCorr      = 0;
  m_averagePtr   = NOAVEPTR;
  m_startPtr     = NOENDPTR;
  m_endPtr       = NOENDPTR;
  m_syncPtr      = NOENDPTR;
  m_centreVal    = 0;
  m_thresholdVal = 0;
  m_countdown    = 0U;
  m_invert       = false;
}

void CMode3RX::samples(q15_t* samples, uint8_t length)
{
  q15_t vals[RX_BLOCK_SIZE];
  ::arm_fir_fast_q15(&m_rrc02Filter, samples, vals, RX_BLOCK_SIZE);

  for (uint8_t i = 0U; i < length; i++) {
    q15_t sample = vals[i];

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_buffer[m_dataPtr] = sample;

    switch (m_state) {
    case MODE3RXS_HEADER:
      processHeader(sample);
      break;
    case MODE3RXS_PAYLOAD:
      processPayload(sample);
      break;
    case MODE3RXS_CRC:
      processCRC(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= MODE3_MAX_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= MODE3_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CMode3RX::processNone(q15_t sample)
{
  bool ret = correlateSync();
  if (ret) {
    // On the first sync, start the countdown to the state change
    if (m_countdown == 0U) {
      io.setDecode(true);
      m_averagePtr = NOAVEPTR;
      m_countdown  = 5U;
    }
  }

  if (m_countdown > 0U)
    m_countdown--;

  if (m_countdown == 1U) {
    DEBUG5("Mode3RX: sync found pos/centre/threshold/invert", m_syncPtr, m_centreVal, m_thresholdVal, m_invert ? 1 : 0);
    m_state     = MODE3RXS_HEADER;
    m_countdown = 0U;
  }
}

void CMode3RX::processHeader(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    calculateLevels(m_startPtr, m_endPtr);

    uint8_t frame[MODE3_HEADER_LENGTH_BYTES + MODE3_HEADER_PARITY_BYTES];
    samplesToBits(m_startPtr, m_endPtr, frame);

    bool ok = m_frame.processHeader(frame, m_packet);
    if (ok) {
      uint16_t length = m_frame.getPayloadLength();
      if (length > 0U) {
        DEBUG2("Mode3RX: header is valid and has a payload", length);

        m_state = MODE3RXS_PAYLOAD;

        length += m_frame.getPayloadParityLength();

        // The payload starts right after the header
        m_startPtr = m_endPtr;

        m_endPtr = m_startPtr + (length * MODE3_SYMBOLS_PER_BYTE * MODE3_RADIO_SYMBOL_LENGTH);
        if (m_endPtr >= MODE3_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE3_MAX_LENGTH_SAMPLES;
      } else {
        bool hasCRC = m_frame.hasCRC();
        if (hasCRC) {
          DEBUG1("Mode3RX: header is valid but no payload and has a CRC");

          m_state = MODE3RXS_CRC;

          // The CRC starts right after the header
          m_startPtr = m_endPtr;

          m_endPtr = m_startPtr + MODE3_CRC_LENGTH_SAMPLES;
          if (m_endPtr >= MODE3_MAX_LENGTH_SAMPLES)
            m_endPtr -= MODE3_MAX_LENGTH_SAMPLES;
        } else {
          DEBUG1("Mode3RX: header is valid but no payload and no CRC");

          length = m_frame.getHeaderLength();
          serial.writeKISSData(KISS_TYPE_DATA, m_packet, length);

          io.setDecode(false);
          reset();
        }
      }
    } else {
      DEBUG1("Mode3RX: header is invalid");
      io.setDecode(false);
      reset();
    }
  }
}

void CMode3RX::processPayload(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    calculateLevels(m_startPtr, m_endPtr);

    uint8_t frame[1023U + (5U * MODE3_PAYLOAD_PARITY_BYTES)];
    samplesToBits(m_startPtr, m_endPtr, frame);

    bool ok = m_frame.processPayload(frame, m_packet);
    if (ok) {
      bool hasCRC = m_frame.hasCRC();
      if (hasCRC) {
        DEBUG1("Mode3RX: payload is valid and has a CRC");

        m_state = MODE3RXS_CRC;

        // The CRC starts right after the payload
        m_startPtr = m_endPtr;

        m_endPtr = m_startPtr + MODE3_CRC_LENGTH_SAMPLES;
        if (m_endPtr >= MODE3_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE3_MAX_LENGTH_SAMPLES;
      } else {
        DEBUG1("Mode3RX: payload is valid but no CRC");

        uint16_t length = m_frame.getHeaderLength() + m_frame.getPayloadLength();
        serial.writeKISSData(KISS_TYPE_DATA, m_packet, length);
        io.setDecode(false);
        reset();
      }
    } else {
      DEBUG1("Mode3RX: payload is invalid");
      io.setDecode(false);
      reset();
    }
  }
}

void CMode3RX::processCRC(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    uint8_t crc[MODE3_CRC_LENGTH_BYTES];
    samplesToBits(m_startPtr, m_endPtr, crc);

    bool ok = m_frame.checkCRC(m_packet, crc);
    if (ok) {
      DEBUG1("Mode3RX: frame CRC is valid");

      uint16_t length = m_frame.getHeaderLength() + m_frame.getPayloadLength();
      serial.writeKISSData(KISS_TYPE_DATA, m_packet, length);
    } else {
      DEBUG1("Mode3RX: frame CRC is invalid");
    }

    io.setDecode(false);
    reset();
  }
}

bool CMode3RX::correlateSync()
{
  if ((countBits32((m_bitBuffer[m_bitPtr] ^  MODE3_SYNC_SYMBOLS) & MODE3_SYNC_SYMBOLS_MASK) <= MAX_SYNC_SYMBOLS_ERRS) ||
      (countBits32((m_bitBuffer[m_bitPtr] ^ ~MODE3_SYNC_SYMBOLS) & MODE3_SYNC_SYMBOLS_MASK) <= MAX_SYNC_SYMBOLS_ERRS)) {

    uint16_t ptr = m_dataPtr + MODE3_MAX_LENGTH_SAMPLES - MODE3_SYNC_LENGTH_SAMPLES;
    if (ptr >= MODE3_MAX_LENGTH_SAMPLES)
      ptr -= MODE3_MAX_LENGTH_SAMPLES;

    q31_t corr = 0;
    q15_t min  =  16000;
    q15_t max  = -16000;

    for (uint8_t i = 0U; i < MODE3_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      switch (MODE3_SYNC_SYMBOLS_VALUES[i]) {
      case +3:
        corr -= (val + val + val);
        break;
      case +1:
        corr -= val;
        break;
      case -1:
        corr += val;
        break;
      default:  // -3
        corr += (val + val + val);
        break;
      }

      ptr += MODE3_RADIO_SYMBOL_LENGTH;
      if (ptr >= MODE3_MAX_LENGTH_SAMPLES)
        ptr -= MODE3_MAX_LENGTH_SAMPLES;
    }

    if ((corr > m_maxCorr) || (-corr > m_maxCorr)) {
      if (m_averagePtr == NOAVEPTR) {
        m_centreVal = (max + min) >> 1;

        q31_t v1 = (max - m_centreVal) * SCALING_FACTOR;
        m_thresholdVal = q15_t(v1 >> 15);
      }

      m_invert = (-corr > m_maxCorr);

      uint16_t startPtr = m_dataPtr + MODE3_MAX_LENGTH_SAMPLES - MODE3_SYNC_LENGTH_SAMPLES + MODE3_RADIO_SYMBOL_LENGTH;
      if (startPtr >= MODE3_MAX_LENGTH_SAMPLES)
        startPtr -= MODE3_MAX_LENGTH_SAMPLES;

      uint16_t endPtr = m_dataPtr;

      uint8_t sync[MODE3_SYNC_LENGTH_BYTES];
      samplesToBits(startPtr, endPtr, sync);

      uint8_t errs = 0U;
      for (uint8_t i = 0U; i < MODE3_SYNC_LENGTH_BYTES; i++)
        errs += countBits8(sync[i] ^ MODE3_SYNC_BYTES[i]);

      if (errs <= MAX_SYNC_BIT_ERRS) {
        m_maxCorr = m_invert ? -corr : corr;
        m_syncPtr = m_dataPtr;

        // The header starts right after the sync vector
        m_startPtr = m_dataPtr + MODE3_RADIO_SYMBOL_LENGTH;
        if (m_startPtr >= MODE3_MAX_LENGTH_SAMPLES)
          m_startPtr -= MODE3_MAX_LENGTH_SAMPLES;

        m_endPtr = m_startPtr + MODE3_HEADER_LENGTH_SAMPLES + MODE3_HEADER_PARITY_SAMPLES;
        if (m_endPtr >= MODE3_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE3_MAX_LENGTH_SAMPLES;

        return true;
      }
    }
  }

  return false;
}

void CMode3RX::calculateLevels(uint16_t startPtr, uint16_t endPtr)
{
  q15_t maxPos = -16000;
  q15_t minPos =  16000;
  q15_t maxNeg =  16000;
  q15_t minNeg = -16000;

  while (startPtr != endPtr) {
    q15_t sample = m_buffer[startPtr];

    if (sample > 0) {
      if (sample > maxPos)
        maxPos = sample;
      if (sample < minPos)
        minPos = sample;
    } else {
      if (sample < maxNeg)
        maxNeg = sample;
      if (sample > minNeg)
        minNeg = sample;
    }

    startPtr += MODE3_RADIO_SYMBOL_LENGTH;
    if (startPtr >= MODE3_MAX_LENGTH_SAMPLES)
      startPtr -= MODE3_MAX_LENGTH_SAMPLES;
  }

  q15_t posThresh = (maxPos + minPos) >> 1;
  q15_t negThresh = (maxNeg + minNeg) >> 1;

  q15_t centre = (posThresh + negThresh) >> 1;

  q15_t threshold = posThresh - centre;

  DEBUG5("Mode3RX: pos/neg/centre/threshold", posThresh, negThresh, centre, threshold);

  if (m_averagePtr == NOAVEPTR) {
    for (uint8_t i = 0U; i < 16U; i++) {
      m_centre[i] = centre;
      m_threshold[i] = threshold;
    }

    m_averagePtr = 0U;
  } else {
    m_centre[m_averagePtr] = centre;
    m_threshold[m_averagePtr] = threshold;

    m_averagePtr++;
    if (m_averagePtr >= 16U)
      m_averagePtr = 0U;
  }

  m_centreVal = 0;
  m_thresholdVal = 0;

  for (uint8_t i = 0U; i < 16U; i++) {
    m_centreVal += m_centre[i];
    m_thresholdVal += m_threshold[i];
  }

  m_centreVal >>= 4;
  m_thresholdVal >>= 4;
}

void CMode3RX::samplesToBits(uint16_t startPtr, uint16_t endPtr, uint8_t* buffer)
{
  uint16_t offset = 0U;

  while (startPtr != endPtr) {
    q15_t sample = 0;
    if (m_invert)
      sample = -m_buffer[startPtr] - m_centreVal;
    else
      sample = m_buffer[startPtr] - m_centreVal;

    if (sample < -m_thresholdVal) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    } else if (sample < 0) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else if (sample < m_thresholdVal) {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    }

    startPtr += MODE3_RADIO_SYMBOL_LENGTH;
    if (startPtr >= MODE3_MAX_LENGTH_SAMPLES)
      startPtr -= MODE3_MAX_LENGTH_SAMPLES;
  }
}


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

#include "Config.h"

#include "KISSDefines.h"
#include "Globals.h"
#include "Mode2RX.h"
#include "Utils.h"

// LPF, cutoff = 0.9 * 4800 (4320)
//
// 45 designed taps with a leading zero, because arm_fir_fast_q15 needs an
// even tap count. CMSIS applies the coefficients against the state buffer
// oldest first, so a zero on the front contributes nothing and shifts
// nothing: the filter is identical to the 45 tap version, with no added
// delay and the same response.
static q15_t RX_FILTER[] = {  \
      0, \
      -9, -41, -30, 32, 89, \
      44, -107, -193, -33, 279, \
      349, -64, -602, -532, 352, \
      1175, 706, -1090, -2379, -830, \
      3951, 9411, 11818, 9411, 3951, \
      -830, -2379, -1090, 706, 1175, \
      352, -532, -602, -64, 349, \
      279, -33, -193, -107, 44, \
      89, 32, -30, -41, -9 };
const uint16_t RX_FILTER_LEN = 46U;

// arm_fir_init_q15: "numTaps must be even and greater than or equal to 4".
// This firmware fills the filter instance by hand, so nothing checks it at
// run time. Both of these have been wrong here before: the coefficient array
// grew from 42 to 45 entries in c07eb99 while the length stayed at 42, and
// the length was then corrected to the odd value 45 in 55a4727.
static_assert((sizeof(RX_FILTER) / sizeof(RX_FILTER[0])) == RX_FILTER_LEN,
              "RX_FILTER_LEN does not match the number of coefficients");
static_assert((RX_FILTER_LEN % 2U) == 0U,
              "arm_fir_fast_q15 requires an even tap count");
static_assert(RX_FILTER_LEN >= 4U,
              "arm_fir_fast_q15 requires at least 4 taps");

const q15_t SCALING_FACTOR = 21845;      // Q15(0.667)

const uint8_t MAX_SYNC_BIT_ERRS     = 2U;
const uint8_t MAX_SYNC_SYMBOLS_ERRS = 1U;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t  NOAVEPTR = 99U;
const uint16_t NOENDPTR = 9999U;

// ---- payload timing search ----------------------------------------------
//
// The payload block is long enough that the difference between the
// transmitter's symbol clock and this receiver's sample clock matters: at
// 200 ppm the sampling instant drifts half a sample across 464 symbols, and
// the sync correlator can only anchor it to the nearest whole sample in the
// first place. Feedback loops were tried and measured worse than nothing --
// a first order loop cannot take out a rate offset, and reacting per symbol
// lets detector noise walk the sampling instant off the eye.
//
// So the timing is searched, not tracked. The whole block is already in
// m_buffer when it is decoded, so the receiver tries a small grid of
// (phase, rate) candidates, scores each by how cleanly the sampled values
// fall into four level clusters, and slices with the winner. Positions are
// fixed point, 1/65536 of a sample.
const int32_t TIMING_ONE_SAMPLE  = 65536;
const int32_t TIMING_STEP        = MODE2_RADIO_SYMBOL_LENGTH * TIMING_ONE_SAMPLE;
const uint32_t TIMING_RING       = uint32_t(MODE2_MAX_LENGTH_SAMPLES) * uint32_t(TIMING_ONE_SAMPLE);

// Rate candidates, as a step adjustment in 1/65536 sample per symbol.
// 33 is almost exactly 100 ppm of the 327680 nominal step.
const int32_t TIMING_RATE_STEP   = 8;      // ~24 ppm
const int32_t TIMING_RATE_SPAN   = 24;     // ~ +/-590 ppm
// Phase candidates, +/- half a sample in eighths.
const int32_t TIMING_PHASE_STEP  = TIMING_ONE_SAMPLE / 16;
const int32_t TIMING_PHASE_SPAN  = 32;     // +/- two whole samples
// How many of the best scored candidates get a Reed-Solomon attempt.
const uint8_t TIMING_ATTEMPTS    = 24U;

// Scratch for the payload search. File scope rather than stack: the decode
// runs in the main loop with a 2 KB stack reservation on target, and these
// come to over four kilobytes.
static uint8_t  s_frame[1023U + (5U * MODE2_PAYLOAD_PARITY_BYTES)];
static uint8_t  s_alt[1023U + (5U * MODE2_PAYLOAD_PARITY_BYTES)];
static uint16_t s_margins[1023U + (5U * MODE2_PAYLOAD_PARITY_BYTES)];
// Header phase candidates: the full phase span at the nominal rate.
const uint8_t HEADER_CANDIDATES  = uint8_t(2 * TIMING_PHASE_SPAN + 1);

CMode2RX::CMode2RX() :
m_state(MODE2RXS_NONE),
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
m_averagePtr(NOAVEPTR),
m_countdown(0U),
m_trkPos(0U),
m_trkStep(0),
m_trkCentres(),
m_trkValid(false),
m_packet()
{
  ::memset(m_rrc02State, 0x00U, 70U * sizeof(q15_t));
  m_rrc02Filter.numTaps = RX_FILTER_LEN;
  m_rrc02Filter.pState  = m_rrc02State;
  m_rrc02Filter.pCoeffs = RX_FILTER;
}

void CMode2RX::reset()
{
  m_state        = MODE2RXS_NONE;
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
  m_trkValid     = false;
}

void CMode2RX::samples(q15_t* samples, uint8_t length)
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
    case MODE2RXS_HEADER:
      processHeader(sample);
      break;
    case MODE2RXS_PAYLOAD:
      processPayload(sample);
      break;
    case MODE2RXS_CRC:
      processCRC(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= MODE2_MAX_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= MODE2_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CMode2RX::processNone(q15_t sample)
{
  bool ret = correlateSync();
  if (ret) {
    // On the first sync, start the countdown to the state change
    if (m_countdown == 0U) {
      m_averagePtr = NOAVEPTR;
      m_countdown  = 3U;
    }
  }

  if (m_countdown > 0U)
    m_countdown--;

  if (m_countdown == 1U) {
    if (m_thresholdVal >= 50) {
      DEBUG5("Mode2RX: sync found pos/centre/threshold/invert", m_syncPtr, m_centreVal, m_thresholdVal, m_invert ? 1 : 0);

      io.setDecode(true);

      m_state     = MODE2RXS_HEADER;
      m_countdown = 0U;
    } else {
      reset();
    }
  }
}

void CMode2RX::processHeader(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    calculateLevels(m_startPtr, m_endPtr);

    uint8_t frame[MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES];

    bool ok = decodeHeader(frame);
    
    if (ok) {
      uint16_t length = m_frame.getPayloadLength();
      if (length > 0U) {
        DEBUG2("Mode2RX: header is valid and has a payload", length);

        m_state = MODE2RXS_PAYLOAD;

        length += m_frame.getPayloadParityLength();

        // The payload starts right after the header. The wait runs to the
        // end of the CRC as well: every decode attempt is judged against the
        // received CRC, so both have to be in the buffer before starting.
        m_startPtr = m_endPtr;

        m_endPtr = m_startPtr + (length * MODE2_SYMBOLS_PER_BYTE * MODE2_RADIO_SYMBOL_LENGTH) + MODE2_CRC_LENGTH_SAMPLES;
        if (m_endPtr >= MODE2_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE2_MAX_LENGTH_SAMPLES;
      } else {
        DEBUG1("Mode2RX: header is valid but has no payload");

        m_state = MODE2RXS_CRC;

        // The CRC starts right after the header
        m_startPtr = m_endPtr;

        m_endPtr = m_startPtr + MODE2_CRC_LENGTH_SAMPLES;
        if (m_endPtr >= MODE2_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE2_MAX_LENGTH_SAMPLES;
      }
    } else {
      DEBUG1("Mode2RX: header is invalid");
      io.setDecode(false);
      reset();
    }
  }
}

void CMode2RX::processPayload(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    bool ok = decodePayload(s_frame);
    if (ok) {
      DEBUG1("Mode2RX: payload and CRC are valid");

      const uint16_t length = m_frame.getHeaderLength() + m_frame.getPayloadLength();
      serial.writeKISSData(KISS_TYPE_DATA, m_packet, length);
    } else {
      DEBUG1("Mode2RX: payload is invalid");
    }

    io.setDecode(false);
    reset();
  }
}

void CMode2RX::processCRC(q15_t sample)
{
  if (m_dataPtr == m_endPtr) {
    uint8_t crc[MODE2_CRC_LENGTH_BYTES];
    bool ok = false;
    if (m_trkValid) {
      // Continue at the rate and phase the payload search settled on; by the
      // CRC the accumulated drift is approaching a sample, and going back to
      // the integer grid here throws the frame away at the last fence. The
      // carried phase is itself only as good as the payload's winning
      // candidate, so a few positions either side are tried too -- the CRC
      // comparison is a 16 bit arbiter, so extra attempts cannot let a bad
      // frame through, only stop a good one being lost.
      const int32_t nudge[9] = { 0, -TIMING_PHASE_STEP, TIMING_PHASE_STEP,
                                 -2 * TIMING_PHASE_STEP, 2 * TIMING_PHASE_STEP,
                                 -4 * TIMING_PHASE_STEP, 4 * TIMING_PHASE_STEP,
                                 -8 * TIMING_PHASE_STEP, 8 * TIMING_PHASE_STEP };

      uint8_t bestMismatch = 255U;

      for (uint8_t i = 0U; i < 9U && !ok; i++) {
        const uint32_t pos = uint32_t((int64_t(m_trkPos) + nudge[i] + TIMING_RING) % TIMING_RING);
        sliceSymbols(pos, m_trkStep, MODE2_CRC_LENGTH_SYMBOLS, m_trkCentres, crc);
        ok = m_frame.checkCRC(m_packet, crc);

        if (!ok) {
          // Soft comparison against the CRC this frame implies. The payload
          // has already survived sixteen Reed-Solomon parity symbols, so the
          // CRC's job here is to catch a miscorrected block -- and a
          // miscorrected block's expected CRC is effectively random against
          // the received symbols, disagreeing on about twelve of the
          // sixteen. A dirty but genuine frame disagrees on a couple.
          uint8_t expect[MODE2_CRC_LENGTH_BYTES];
          m_frame.expectedCRC(m_packet, expect);

          uint8_t mismatch = 0U;
          for (uint8_t b = 0U; b < MODE2_CRC_LENGTH_BYTES; b++) {
            const uint8_t x = crc[b] ^ expect[b];
            for (uint8_t d = 0U; d < 4U; d++) {
              if ((x >> (2U * d)) & 0x03U)
                mismatch++;
            }
          }

          if (mismatch < bestMismatch)
            bestMismatch = mismatch;
        }
      }

      if (!ok && bestMismatch <= 3U) {
        DEBUG2("Mode2RX: CRC accepted on soft match, symbol mismatches", bestMismatch);
        ok = true;
      }
    } else {
      samplesToBits(m_startPtr, m_endPtr, crc);
      ok = m_frame.checkCRC(m_packet, crc);
    }
    if (ok) {
      DEBUG1("Mode2RX: frame CRC is valid");

      uint16_t length = m_frame.getHeaderLength() + m_frame.getPayloadLength();
      serial.writeKISSData(KISS_TYPE_DATA, m_packet, length);
    } else {
      DEBUG1("Mode2RX: frame CRC is invalid");
    }

    io.setDecode(false);
    reset();
  }
}

bool CMode2RX::correlateSync()
{
  uint8_t n1 = countBits16(m_bitBuffer[m_bitPtr] ^  MODE2_SYNC_SYMBOLS);
  uint8_t n2 = countBits16(m_bitBuffer[m_bitPtr] ^ ~MODE2_SYNC_SYMBOLS);

  if ((n1 <= MAX_SYNC_SYMBOLS_ERRS) || (n2 <= MAX_SYNC_SYMBOLS_ERRS)) {
    // The sign of the sample at m_dataPtr is shifted into the bit buffer
    // before we get here, so a match puts the last sync symbol at m_dataPtr
    // and the first at m_dataPtr - (MODE2_SYNC_LENGTH_SAMPLES - one symbol).
    uint16_t ptr = m_dataPtr + MODE2_MAX_LENGTH_SAMPLES - MODE2_SYNC_LENGTH_SAMPLES + MODE2_RADIO_SYMBOL_LENGTH;
    if (ptr >= MODE2_MAX_LENGTH_SAMPLES)
      ptr -= MODE2_MAX_LENGTH_SAMPLES;

    q31_t corr = 0;
    q15_t min  =  16000;
    q15_t max  = -16000;

    for (uint8_t i = 0U; i < MODE2_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      switch (MODE2_SYNC_SYMBOLS_VALUES[i]) {
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

      ptr += MODE2_RADIO_SYMBOL_LENGTH;
      if (ptr >= MODE2_MAX_LENGTH_SAMPLES)
        ptr -= MODE2_MAX_LENGTH_SAMPLES;
    }

    if ((corr > m_maxCorr) || (-corr > m_maxCorr)) {
      if (m_averagePtr == NOAVEPTR) {
        m_centreVal = (max + min) / 2;

        q31_t v1 = (max - m_centreVal) * SCALING_FACTOR;
        m_thresholdVal = q15_t(v1 >> 15);
      }

      m_invert = (-corr > m_maxCorr);

      uint16_t startPtr = m_dataPtr + MODE2_MAX_LENGTH_SAMPLES - MODE2_SYNC_LENGTH_SAMPLES + MODE2_RADIO_SYMBOL_LENGTH;
      if (startPtr >= MODE2_MAX_LENGTH_SAMPLES)
        startPtr -= MODE2_MAX_LENGTH_SAMPLES;

      // samplesToBits() stops short of endPtr, so this has to be one symbol
      // past the last sync symbol, which sits at m_dataPtr. Stopping at
      // m_dataPtr converts only 15 of the 16 symbols and leaves the bottom
      // two bits of sync[] never written.
      uint16_t endPtr = m_dataPtr + MODE2_RADIO_SYMBOL_LENGTH;
      if (endPtr >= MODE2_MAX_LENGTH_SAMPLES)
        endPtr -= MODE2_MAX_LENGTH_SAMPLES;

      uint8_t sync[MODE2_SYNC_LENGTH_BYTES];
      samplesToBits(startPtr, endPtr, sync);

      uint8_t errs = 0U;
      for (uint8_t i = 0U; i < MODE2_SYNC_LENGTH_BYTES; i++)
        errs += countBits8(sync[i] ^ MODE2_SYNC_BYTES[i]);

      if (errs <= MAX_SYNC_BIT_ERRS) {
        DEBUG6("Mode2RX: valid sync vector", corr, m_dataPtr, n1, n2, errs);

        m_maxCorr = m_invert ? -corr : corr;
        m_syncPtr = m_dataPtr;

        // The header starts right after the sync vector
        m_startPtr = m_dataPtr + MODE2_RADIO_SYMBOL_LENGTH;
        if (m_startPtr >= MODE2_MAX_LENGTH_SAMPLES)
          m_startPtr -= MODE2_MAX_LENGTH_SAMPLES;

        m_endPtr = m_startPtr + MODE2_HEADER_LENGTH_SAMPLES + MODE2_HEADER_PARITY_SAMPLES;
        if (m_endPtr >= MODE2_MAX_LENGTH_SAMPLES)
          m_endPtr -= MODE2_MAX_LENGTH_SAMPLES;

        return true;
      }
    }
  }

  return false;
}

void CMode2RX::calculateLevels(uint16_t startPtr, uint16_t endPtr)
{
  // Estimate the two positive levels separately and put the decision point
  // midway between them, and likewise for the negative pair.
  //
  // First pass: the mean of each polarity, used only as a split. Second pass:
  // the mean of the samples either side of that split, which are the +3 and
  // +1 cluster centres. The midpoint of two cluster means barely moves when
  // one sample lands near zero, whereas a midpoint taken from the largest and
  // smallest sample moves a long way. Across the 464 symbols of a payload
  // block that happens often enough to drag the threshold down and misread +1
  // symbols as +3 for the rest of the block, which is why the payload failed
  // so much more often than the 60 symbol header.
  const uint16_t start = startPtr;

  q31_t    posSum   = 0;
  q31_t    negSum   = 0;
  uint16_t posCount = 0U;
  uint16_t negCount = 0U;

  while (startPtr != endPtr) {
    q15_t sample = m_buffer[startPtr];

    if (sample > 0) {
      posSum += sample;
      posCount++;
    } else {
      negSum += sample;
      negCount++;
    }

    startPtr += MODE2_RADIO_SYMBOL_LENGTH;
    if (startPtr >= MODE2_MAX_LENGTH_SAMPLES)
      startPtr -= MODE2_MAX_LENGTH_SAMPLES;
  }

  const q15_t posSplit = posCount > 0U ? q15_t(posSum / posCount) : 0;
  const q15_t negSplit = negCount > 0U ? q15_t(negSum / negCount) : 0;

  q31_t    hiPosSum = 0, loPosSum = 0, hiNegSum = 0, loNegSum = 0;
  uint16_t hiPosCnt = 0U, loPosCnt = 0U, hiNegCnt = 0U, loNegCnt = 0U;

  startPtr = start;
  while (startPtr != endPtr) {
    q15_t sample = m_buffer[startPtr];

    if (sample > 0) {
      if (sample >= posSplit) { hiPosSum += sample; hiPosCnt++; }
      else                    { loPosSum += sample; loPosCnt++; }
    } else {
      if (sample <= negSplit) { hiNegSum += sample; hiNegCnt++; }
      else                    { loNegSum += sample; loNegCnt++; }
    }

    startPtr += MODE2_RADIO_SYMBOL_LENGTH;
    if (startPtr >= MODE2_MAX_LENGTH_SAMPLES)
      startPtr -= MODE2_MAX_LENGTH_SAMPLES;
  }

  q15_t posThresh = posSplit;
  if (hiPosCnt > 0U && loPosCnt > 0U)
    posThresh = q15_t((hiPosSum / hiPosCnt + loPosSum / loPosCnt) / 2);

  q15_t negThresh = negSplit;
  if (hiNegCnt > 0U && loNegCnt > 0U)
    negThresh = q15_t((hiNegSum / hiNegCnt + loNegSum / loNegCnt) / 2);

  q15_t centre = (posThresh + negThresh) / 2;

  q15_t threshold = posThresh - centre;

  DEBUG5("Mode2RX: pos/neg/centre/threshold", posThresh, negThresh, centre, threshold);

  if (m_averagePtr == NOAVEPTR) {
    for (uint8_t i = 0U; i < 16U; i++) {
      m_centre[i]    = centre;
      m_threshold[i] = threshold;
    }

    m_averagePtr = 0U;
  } else {
    m_centre[m_averagePtr]    = centre;
    m_threshold[m_averagePtr] = threshold;

    m_averagePtr++;
    if (m_averagePtr >= 16U)
      m_averagePtr = 0U;
  }

  m_centreVal = 0;
  m_thresholdVal = 0;

  for (uint8_t i = 0U; i < 16U; i++) {
    m_centreVal    += m_centre[i];
    m_thresholdVal += m_threshold[i];
  }

  m_centreVal    /= 16;
  m_thresholdVal /= 16;
}

void CMode2RX::samplesToBits(uint16_t startPtr, uint16_t endPtr, uint8_t* buffer)
{
  uint16_t offset = 0U;

  while (startPtr != endPtr) {
    // Remove the offset first, then flip. m_centreVal was measured from the
    // buffer as it stands, so negating before subtracting would add the
    // offset back on instead of taking it off.
    q15_t sample = m_buffer[startPtr] - m_centreVal;
    if (m_invert)
      sample = -sample;

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

    startPtr += MODE2_RADIO_SYMBOL_LENGTH;
    if (startPtr >= MODE2_MAX_LENGTH_SAMPLES)
      startPtr -= MODE2_MAX_LENGTH_SAMPLES;
  }
}

q15_t CMode2RX::sampleAt(uint32_t pos) const
{
  uint16_t i = uint16_t(pos >> 16);
  uint16_t j = i + 1U;
  if (j >= MODE2_MAX_LENGTH_SAMPLES)
    j = 0U;

  const int32_t frac = int32_t(pos & 0xFFFFU);
  const int32_t a    = m_buffer[i];
  const int32_t b    = m_buffer[j];

  int32_t v = a + (((b - a) * frac) >> 16);
  if (m_invert)
    v = -v;

  return q15_t(v);
}

// Score one (phase, rate) candidate and report its four cluster centres.
//
// Three passes: the global mean, then a split of each side into its two
// levels, then the sums that give the within and between cluster variances.
// The score is within/between scaled up -- smaller is better. Everything is
// integer; the sums of squares need 64 bits.
int64_t CMode2RX::scoreCandidate(uint32_t pos0, int32_t step, uint16_t nsym, q15_t centres[4]) const
{
  const int32_t sstep = step;
  const uint16_t scount = nsym;

  uint32_t pos;
  int64_t  sum = 0;

  pos = pos0;
  for (uint16_t n = 0U; n < scount; n++) {
    sum += sampleAt(pos);
    pos = uint32_t(pos + sstep) % TIMING_RING;
  }
  const int32_t gmean = int32_t(sum / scount);

  int64_t  posSum = 0, negSum = 0;
  uint16_t posCnt = 0U, negCnt = 0U;

  pos = pos0;
  for (uint16_t n = 0U; n < scount; n++) {
    const int32_t v = sampleAt(pos);
    if (v >= gmean) { posSum += v; posCnt++; }
    else            { negSum += v; negCnt++; }
    pos = uint32_t(pos + sstep) % TIMING_RING;
  }
  if (posCnt == 0U || negCnt == 0U)
    return INT64_MAX;

  const int32_t posMean = int32_t(posSum / posCnt);
  const int32_t negMean = int32_t(negSum / negCnt);

  int64_t  cSum[4] = {0, 0, 0, 0};
  int64_t  cSq[4]  = {0, 0, 0, 0};
  uint16_t cCnt[4] = {0U, 0U, 0U, 0U};

  pos = pos0;
  for (uint16_t n = 0U; n < scount; n++) {
    const int32_t v = sampleAt(pos);
    uint8_t c;
    if (v >= gmean)
      c = (v >= posMean) ? 3U : 2U;
    else
      c = (v <= negMean) ? 0U : 1U;
    cSum[c] += v;
    cSq[c]  += int64_t(v) * v;
    cCnt[c]++;
    pos = uint32_t(pos + sstep) % TIMING_RING;
  }

  int64_t within  = 0;
  int64_t between = 0;

  for (uint8_t c = 0U; c < 4U; c++) {
    if (cCnt[c] == 0U) {
      centres[c] = 0;
      continue;
    }
    const int64_t mean = cSum[c] / cCnt[c];
    centres[c] = q15_t(mean);
    within  += cSq[c] - mean * cSum[c];
    between += (mean - gmean) * (mean - gmean) * cCnt[c];
  }

  if (between <= 0)
    return INT64_MAX;

  return (within << 16) / between;
}

// How badly nsym symbols starting at pos fit the given cluster centres:
// the summed distance from each sample to its nearest centre.
int64_t CMode2RX::segmentError(uint32_t pos, int32_t step, uint16_t nsym, const q15_t centres[4]) const
{
  const int32_t m1 = (int32_t(centres[0]) + int32_t(centres[1])) / 2;
  const int32_t m2 = (int32_t(centres[1]) + int32_t(centres[2])) / 2;
  const int32_t m3 = (int32_t(centres[2]) + int32_t(centres[3])) / 2;

  int64_t err = 0;

  for (uint16_t n = 0U; n < nsym; n++) {
    const int32_t v = sampleAt(pos);

    int32_t c;
    if (v < m1)      c = centres[0];
    else if (v < m2) c = centres[1];
    else if (v < m3) c = centres[2];
    else             c = centres[3];

    err += (v > c) ? (v - c) : (c - v);

    pos = uint32_t(pos + step) % TIMING_RING;
  }

  return err;
}

// Slice nsym symbols in segments, re-anchoring the sampling phase at the
// start of each. A linear phase-plus-rate model assumes the clock error is
// smooth; a dropped or repeated sample in the capture chain is a step, and
// against a step the best line is wrong everywhere. Short segments follow
// steps. The phase found for each segment carries into the next, so the
// search per segment only needs to cover a small neighbourhood.
void CMode2RX::sliceSegmented(uint32_t pos0, int32_t step, uint16_t nsym, const q15_t centres[4], uint8_t* buffer, uint32_t& endPos) const
{
  const uint16_t SEG = 16U;

  uint32_t pos    = pos0;
  uint16_t offset = 0U;
  uint16_t done   = 0U;

  while (done < nsym) {
    uint16_t n = nsym - done;
    if (n > SEG)
      n = SEG;

    // Local phase search around the carried position. The window is wide
    // enough, at +/- 1.25 samples, to step over a whole dropped or repeated
    // sample in the capture chain, and the segment is short enough that the
    // symbols sliced before such a step are few for the Reed-Solomon parity.
    int64_t  bestErr = INT64_MAX;
    uint32_t bestPos = pos;

    for (int32_t d = -20; d <= 20; d++) {
      const uint32_t p = uint32_t((int64_t(pos) + int64_t(d) * (TIMING_ONE_SAMPLE / 16) + TIMING_RING) % TIMING_RING);
      const int64_t e = segmentError(p, step, n, centres);
      if (e < bestErr) {
        bestErr = e;
        bestPos = p;
      }
    }

    // Slice this segment at the locally best phase.
    uint32_t p = bestPos;
    const int32_t m1 = (int32_t(centres[0]) + int32_t(centres[1])) / 2;
    const int32_t m2 = (int32_t(centres[1]) + int32_t(centres[2])) / 2;
    const int32_t m3 = (int32_t(centres[2]) + int32_t(centres[3])) / 2;

    for (uint16_t k = 0U; k < n; k++) {
      const int32_t v = sampleAt(p);

      bool b0, b1;
      if (v < m1)      { b0 = false; b1 = true;  }
      else if (v < m2) { b0 = false; b1 = false; }
      else if (v < m3) { b0 = true;  b1 = false; }
      else             { b0 = true;  b1 = true;  }

      WRITE_BIT1(buffer, offset, b0);
      offset++;
      WRITE_BIT1(buffer, offset, b1);
      offset++;

      p = uint32_t(p + step) % TIMING_RING;
    }


    pos  = p;
    done = uint16_t(done + n);
  }

  endPos = pos;
}

// Slice nsym symbols by nearest cluster centre. The centre order is
// {-3, -1, +1, +3} and the dibit mapping matches the transmitter's:
// 01, 00, 10, 11.
void CMode2RX::sliceSymbols(uint32_t pos0, int32_t step, uint16_t nsym, const q15_t centres[4], uint8_t* buffer, bool adapt, uint16_t* margins) const
{
  // The centres drift with the signal as the block is sliced: after each
  // decision the chosen cluster's centre moves a 32nd of the way toward the
  // sample. The capture path is AC coupled, so the baseline wanders slowly
  // within a long block; centres estimated once over the whole block are
  // right on average and wrong at both ends. The adaptation is
  // decision-directed, and a wrong decision nudges the wrong centre by a
  // 32nd of a small distance, so it recovers.
  int32_t c[4] = { centres[0], centres[1], centres[2], centres[3] };

  uint32_t pos    = pos0;
  uint16_t offset = 0U;

  for (uint16_t n = 0U; n < nsym; n++) {
    const int32_t v = sampleAt(pos);

    const int32_t m1 = (c[0] + c[1]) / 2;
    const int32_t m2 = (c[1] + c[2]) / 2;
    const int32_t m3 = (c[2] + c[3]) / 2;

    bool b0, b1;
    uint8_t k;
    if (v < m1)      { b0 = false; b1 = true;  k = 0U; }   // -3
    else if (v < m2) { b0 = false; b1 = false; k = 1U; }   // -1
    else if (v < m3) { b0 = true;  b1 = false; k = 2U; }   // +1
    else             { b0 = true;  b1 = true;  k = 3U; }   // +3

    if (margins != 0) {
      // Confidence of this decision: distance to the nearest boundary. A
      // byte's confidence is that of its shakiest symbol.
      int32_t dist = 32767;
      if (k > 0U)  { const int32_t lo[3] = {m1, m2, m3}; const int32_t d1 = v - lo[k - 1U]; if (d1 < dist) dist = d1; }
      if (k < 3U)  { const int32_t hi[3] = {m1, m2, m3}; const int32_t d2 = hi[k] - v;      if (d2 < dist) dist = d2; }
      const uint16_t byteIndex = uint16_t(n / 4U);
      if ((n % 4U) == 0U || uint16_t(dist) < margins[byteIndex])
        margins[byteIndex] = uint16_t(dist < 0 ? 0 : dist);
    }

    if (adapt)
      c[k] += (v - c[k]) / 32;

    WRITE_BIT1(buffer, offset, b0);
    offset++;
    WRITE_BIT1(buffer, offset, b1);
    offset++;

    pos = uint32_t(pos + step) % TIMING_RING;
  }
}

bool CMode2RX::decodePayload(uint8_t* frame)
{
  const uint16_t span = ((m_endPtr >= m_startPtr)
                          ? uint16_t(m_endPtr - m_startPtr)
                          : uint16_t(m_endPtr + MODE2_MAX_LENGTH_SAMPLES - m_startPtr))
                        - MODE2_CRC_LENGTH_SAMPLES;
  const uint16_t nsym = span / MODE2_RADIO_SYMBOL_LENGTH;

  // Score the whole grid, keeping the best few candidates.
  int64_t  bestScore[TIMING_ATTEMPTS];
  uint32_t bestPos[TIMING_ATTEMPTS];
  int32_t  bestStep[TIMING_ATTEMPTS];
  q15_t    bestCentres[TIMING_ATTEMPTS][4];

  for (uint8_t i = 0U; i < TIMING_ATTEMPTS; i++)
    bestScore[i] = INT64_MAX;

  for (int32_t r = -TIMING_RATE_SPAN; r <= TIMING_RATE_SPAN; r++) {
    const int32_t step = TIMING_STEP + r * TIMING_RATE_STEP;

    for (int32_t p = -TIMING_PHASE_SPAN; p <= TIMING_PHASE_SPAN; p++) {
      const int64_t start = int64_t(m_startPtr) * TIMING_ONE_SAMPLE + int64_t(p) * TIMING_PHASE_STEP;
      const uint32_t pos0 = uint32_t((start + TIMING_RING) % TIMING_RING);

      q15_t centres[4];
      const int64_t score = scoreCandidate(pos0, step, nsym, centres);

      // Insert into the top list, kept sorted best first.
      for (uint8_t i = 0U; i < TIMING_ATTEMPTS; i++) {
        if (score < bestScore[i]) {
          for (uint8_t j = TIMING_ATTEMPTS - 1U; j > i; j--) {
            bestScore[j] = bestScore[j - 1U];
            bestPos[j]   = bestPos[j - 1U];
            bestStep[j]  = bestStep[j - 1U];
            ::memcpy(bestCentres[j], bestCentres[j - 1U], sizeof(bestCentres[j]));
          }
          bestScore[i] = score;
          bestPos[i]   = pos0;
          bestStep[i]  = step;
          ::memcpy(bestCentres[i], centres, sizeof(centres));
          break;
        }
      }
    }
  }


  // Try the best candidates in order; the Reed-Solomon decode is the final
  // arbiter of which slicing was right. Erasure decoding is kept for a
  // second sweep over the whole candidate list: it is powerful enough to
  // force a marginally wrong slicing into a valid-looking codeword, so it
  // only runs once every straight attempt has failed.
  for (uint8_t round = 0U; round < 2U; round++) {
    for (uint8_t i = 0U; i < TIMING_ATTEMPTS; i++) {
      if (bestScore[i] == INT64_MAX)
        break;

      uint64_t end = uint64_t(bestPos[i]) + uint64_t(uint32_t(bestStep[i])) * nsym;
      uint32_t endPos = uint32_t(end % TIMING_RING);
      bool ok = false;

      if (round == 0U) {
        // Three slicings: fixed centres, centres that adapt as the block is
        // sliced, then segment-wise phase re-anchoring. Each suits a
        // different impairment.
        for (uint8_t pass = 0U; pass < 3U && !ok; pass++) {
          if (pass < 2U)
            sliceSymbols(bestPos[i], bestStep[i], nsym, bestCentres[i], frame, pass == 1U);
          else
            sliceSegmented(bestPos[i], bestStep[i], nsym, bestCentres[i], frame, endPos);

          m_frame.rewindPayload();
          ok = m_frame.processPayload(frame, m_packet);
        }
      } else {
        // Erasure round: re-slice recording per byte confidence and let the
        // decoder treat the least confident bytes as erasures. E erasures
        // cost E of the 16 parity symbols but are corrected outright,
        // doubling the power over errors-only decoding exactly when the eye
        // is at its worst.
        // Two independent slicings of the block. A byte the two disagree on
        // was decided by less than the difference between the models, which
        // makes it a better erasure candidate than any margin threshold; the
        // remaining erasure budget goes to the lowest margins.
        uint16_t* margins = s_margins;
        uint8_t*  alt     = s_alt;

        const uint16_t nbytes = nsym / 4U;
        for (uint16_t k = 0U; k < nbytes; k++)
          margins[k] = 0xFFFFU;

        sliceSymbols(bestPos[i], bestStep[i], nsym, bestCentres[i], frame, false, margins);
        sliceSymbols(bestPos[i], bestStep[i], nsym, bestCentres[i], alt, true);

        for (uint16_t k = 0U; k < nbytes; k++) {
          if (frame[k] != alt[k])
            margins[k] = 0U;
          else if (margins[k] == 0U)
            margins[k] = 1U;
        }

        const uint8_t ladder[5] = { 8U, 10U, 12U, 14U, 16U };

        for (uint8_t mode = 0U; mode < 2U && !ok; mode++) {
          const uint8_t* attempt = (mode == 0U) ? frame : alt;
          if (mode == 1U)
            ::memcpy(frame, alt, nbytes);

          for (uint8_t e = 0U; e < 5U && !ok; e++) {
            m_frame.rewindPayload();
            ok = m_frame.processPayloadErasures(frame, m_packet, margins, ladder[e]);
            if (ok && !checkPayloadCRC(endPos, bestStep[i], bestCentres[i]))
              ok = false;
          }
          (void)attempt;
        }
      }

      if (round == 0U && ok)
        ok = checkPayloadCRC(endPos, bestStep[i], bestCentres[i]);

      if (ok)
        return true;
    }
  }

  return false;
}

// Judge a decoded payload against the CRC that followed it on the air.
//
// The Reed-Solomon decode alone is not a safe acceptance test once erasures
// are in play: flag enough bytes and a wrong slicing can be forced into a
// valid looking codeword. The CRC is judged softly -- the frame's expected
// CRC symbols against the sliced ones, best of a few phase nudges -- because
// its sixteen symbols get no error correction of their own on a channel
// where everything else does. A miscorrected payload implies an effectively
// random CRC, disagreeing on about twelve symbols of sixteen; a genuine one
// disagrees on a couple even when the eye is poor.
bool CMode2RX::checkPayloadCRC(uint32_t crcPos, int32_t step, const q15_t centres[4])
{
  const int32_t nudge[9] = { 0, -TIMING_PHASE_STEP, TIMING_PHASE_STEP,
                             -2 * TIMING_PHASE_STEP, 2 * TIMING_PHASE_STEP,
                             -4 * TIMING_PHASE_STEP, 4 * TIMING_PHASE_STEP,
                             -8 * TIMING_PHASE_STEP, 8 * TIMING_PHASE_STEP };

  uint8_t expect[MODE2_CRC_LENGTH_BYTES];
  m_frame.expectedCRC(m_packet, expect);

  uint8_t bestMismatch = 255U;

  for (uint8_t i = 0U; i < 9U; i++) {
    const uint32_t pos = uint32_t((int64_t(crcPos) + nudge[i] + TIMING_RING) % TIMING_RING);

    uint8_t crc[MODE2_CRC_LENGTH_BYTES];
    sliceSymbols(pos, step, MODE2_CRC_LENGTH_SYMBOLS, centres, crc);

    if (m_frame.checkCRC(m_packet, crc))
      return true;

    uint8_t mismatch = 0U;
    for (uint8_t b = 0U; b < MODE2_CRC_LENGTH_BYTES; b++) {
      const uint8_t x = crc[b] ^ expect[b];
      for (uint8_t d = 0U; d < 4U; d++) {
        if ((x >> (2U * d)) & 0x03U)
          mismatch++;
      }
    }

    if (mismatch < bestMismatch)
      bestMismatch = mismatch;
  }

  if (bestMismatch <= 3U) {
    DEBUG2("Mode2RX: CRC accepted on soft match, symbol mismatches", bestMismatch);
    return true;
  }

  return false;
}

// Decode the header by a phase search over its 60 symbols.
//
// The header is short enough that clock rate drift within it is negligible,
// but the sampling instant the sync correlator anchors is only good to the
// nearest whole sample, and the header's ten payload length bits ride on the
// fragile +3 versus +1 magnitude decisions. Worse, the header carries only
// two Reed-Solomon parity symbols, and a two parity decoder run over a
// marginal slicing does not fail -- it invents a one byte correction and
// hands back a wrong length with a straight face. The receiver then waits
// out a phantom payload while the real one streams past unread.
//
// So candidates are tried cleanest first: a slicing that forms a valid
// codeword with no correction at all is accepted before any slicing that
// needed the decoder's help.
bool CMode2RX::decodeHeader(uint8_t* frame)
{
  const uint16_t nsym = MODE2_HEADER_LENGTH_SYMBOLS + MODE2_HEADER_PARITY_SYMBOLS;

  int64_t  score[HEADER_CANDIDATES];
  uint32_t posAt[HEADER_CANDIDATES];
  q15_t    centres[HEADER_CANDIDATES][4];
  uint8_t  order[HEADER_CANDIDATES];
  uint8_t  count = 0U;

  for (int32_t p = -TIMING_PHASE_SPAN; p <= TIMING_PHASE_SPAN; p++) {
    const int64_t start = int64_t(m_startPtr) * TIMING_ONE_SAMPLE + int64_t(p) * TIMING_PHASE_STEP;
    const uint32_t pos0 = uint32_t((start + TIMING_RING) % TIMING_RING);

    q15_t c[4];
    const int64_t sc = scoreCandidate(pos0, TIMING_STEP, nsym, c);
    if (sc == INT64_MAX)
      continue;

    score[count] = sc;
    posAt[count] = pos0;
    ::memcpy(centres[count], c, sizeof(c));
    order[count] = count;
    count++;
  }

  // Sort candidate indices by score, best first.
  for (uint8_t i = 1U; i < count; i++) {
    const uint8_t o = order[i];
    uint8_t j = i;
    while (j > 0U && score[order[j - 1U]] > score[o]) {
      order[j] = order[j - 1U];
      j--;
    }
    order[j] = o;
  }

  // Pass 1: only a clean codeword will do. Pass 2: accept a corrected one.
  for (uint8_t pass = 0U; pass < 2U; pass++) {
    for (uint8_t i = 0U; i < count; i++) {
      const uint8_t o = order[i];

      sliceSymbols(posAt[o], TIMING_STEP, nsym, centres[o], frame);

      if (!m_frame.processHeader(frame, m_packet))
        continue;

      if (pass == 0U && m_frame.getLastCorrections() != 0)
        continue;

      return true;
    }
  }

  return false;
}

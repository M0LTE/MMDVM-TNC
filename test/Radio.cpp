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

#include "Config.h"
#include "Globals.h"

#include "Radio.h"
#include "shim/TestHooks.h"

#include <cmath>

namespace radio {

  std::vector<uint8_t> sampleUIFrame(const char* info)
  {
    std::vector<uint8_t> f;

    /* Destination: TEST-0, command. */
    const char* dest = "TEST  ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(dest[i]) << 1);
    f.push_back(0xE0U);                 /* C=1, R bits set, SSID 0, not last */

    /* Source: M0LTE-1. */
    const char* src = "M0LTE ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(src[i]) << 1);
    f.push_back(0x62U | 0x01U);         /* R bits set, SSID 1, end of address */

    f.push_back(0x03U);                 /* UI */
    f.push_back(0xF0U);                 /* no layer 3 */

    for (const char* p = info; *p != '\0'; p++)
      f.push_back(uint8_t(*p));

    return f;
  }

  std::vector<uint8_t> rawPayload(size_t length)
  {
    /* Deliberately not AX.25 shaped, so CIL2PTX picks a type 0 header and the
       bytes are carried verbatim. Byte 13 must have bit 0 clear for that. */
    std::vector<uint8_t> p;
    p.reserve(length);

    uint8_t x = 0x5AU;
    for (size_t i = 0U; i < length; i++) {
      x = uint8_t(x * 31U + 17U);
      p.push_back(i == 13U ? uint8_t(x & 0xFEU) : x);
    }

    return p;
  }

  /* ---- reference modulator -------------------------------------------- *
   *
   * A deliberate second implementation of the on-air format, so that a test
   * can transmit something the firmware never would. The pulse filter is the
   * same Gaussian BT 0.6 convolved with a five sample step that Mode2TX.cpp
   * uses; it is repeated here rather than shared so that this stays a
   * statement of the wire format rather than a mirror of the transmitter.
   */
  static const q15_t REF_PULSE_FILTER[45] = {
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 17, 319, 2659, 10668,
    22736, 30728, 32767, 30728, 22736,
    10668, 2659, 319, 17, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  };

  static const unsigned REF_L           = 5U;   /* MODE2_RADIO_SYMBOL_LENGTH */
  static const unsigned REF_PHASE_LEN   = 9U;   /* 45 taps / L              */

  std::vector<int8_t> bytesToSymbols(const std::vector<uint8_t>& bytes)
  {
    std::vector<int8_t> syms;
    syms.reserve(bytes.size() * 4U);

    for (size_t i = 0U; i < bytes.size(); i++) {
      uint8_t c = bytes[i];

      for (unsigned k = 0U; k < 4U; k++, c <<= 2) {
        switch (c & 0xC0U) {
          case 0xC0U: syms.push_back(+3); break;   /* 11 -> LEVELA */
          case 0x80U: syms.push_back(+1); break;   /* 10 -> LEVELB */
          case 0x00U: syms.push_back(-1); break;   /* 00 -> LEVELC */
          default:    syms.push_back(-3); break;   /* 01 -> LEVELD */
        }
      }
    }

    return syms;
  }

  std::vector<uint16_t> modulateSymbols(const std::vector<int8_t>& symbols)
  {
    /* Mode2TX.cpp: LEVELA 1362, LEVELB 454, scaled by m_level (128 * 128). */
    const int32_t LEVEL3 = (1362 * (MODE2_TX_LEVEL * 128)) >> 15;
    const int32_t LEVEL1 = (454  * (MODE2_TX_LEVEL * 128)) >> 15;

    /* Pad so the filter runs in and out cleanly. */
    std::vector<int32_t> amp;
    amp.reserve(symbols.size() + 2U * REF_PHASE_LEN);

    for (unsigned i = 0U; i < REF_PHASE_LEN; i++)
      amp.push_back(0);

    for (size_t i = 0U; i < symbols.size(); i++) {
      switch (symbols[i]) {
        case +3: amp.push_back(+LEVEL3); break;
        case +1: amp.push_back(+LEVEL1); break;
        case -1: amp.push_back(-LEVEL1); break;
        default: amp.push_back(-LEVEL3); break;
      }
    }

    for (unsigned i = 0U; i < REF_PHASE_LEN; i++)
      amp.push_back(0);

    /* Polyphase interpolate by L, matching arm_fir_interpolate_q15. */
    std::vector<uint16_t> out;
    out.reserve(amp.size() * REF_L);

    for (size_t n = REF_PHASE_LEN - 1U; n < amp.size(); n++) {
      for (unsigned p = 0U; p < REF_L; p++) {
        int64_t acc = 0;

        for (unsigned j = 0U; j < REF_PHASE_LEN; j++)
          acc += int64_t(amp[n - (REF_PHASE_LEN - 1U) + j]) * int64_t(REF_PULSE_FILTER[j * REF_L + p]);

        int32_t s = int32_t(acc >> 15);
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;

        int32_t adc = int32_t(MID) + s;
        if (adc < 0)    adc = 0;
        if (adc > 4095) adc = 4095;

        out.push_back(uint16_t(adc));
      }
    }

    return out;
  }

  std::vector<uint8_t> onAirBytes(const std::vector<uint8_t>& payload, unsigned preambleBytes)
  {
    std::vector<uint8_t> bytes;

    for (unsigned i = 0U; i < preambleBytes; i++)
      bytes.push_back(MODE2_PREAMBLE_BYTE);

    for (unsigned i = 0U; i < MODE2_SYNC_LENGTH_BYTES; i++)
      bytes.push_back(MODE2_SYNC_BYTES[i]);

    CIL2PTX      frame;
    uint8_t      encoded[2000U];
    const uint16_t len = frame.process(payload.empty() ? NULL : &payload[0], uint16_t(payload.size()), encoded);

    for (uint16_t i = 0U; i < len; i++)
      bytes.push_back(encoded[i]);

    for (unsigned i = 0U; i < 10U; i++)
      bytes.push_back(MODE2_PREAMBLE_BYTE);

    return bytes;
  }

  unsigned syncSymbolOffset(unsigned preambleBytes)
  {
    return preambleBytes * 4U;
  }

  std::vector<uint16_t> modulate(const uint8_t* payload, uint16_t length, uint8_t txDelay)
  {
    m_mode   = 2U;
    m_duplex = true;              /* io.canTX() unconditionally true */
    m_tx     = false;

    mode2TX.setTXDelay(txDelay);

    hooks::g_dacOut.clear();

    const uint8_t rc = mode2TX.writeData(payload, length);
    if (rc != 0U)
      return std::vector<uint16_t>();

    /* Run the transmitter until the DAC has been quiet for a good while. The
       trailing silence is the TX tail, which the firmware emits itself. */
    const unsigned MAX_TICKS  = 24000U * 6U;
    const unsigned QUIET_STOP = 2400U;

    unsigned quiet = 0U;
    bool     seen  = false;

    for (unsigned t = 0U; t < MAX_TICKS && quiet < QUIET_STOP; t++) {
      mode2TX.process();
      io.interrupt();

      const uint16_t s = hooks::g_dacOut.back();
      if (s != MID) {
        seen  = true;
        quiet = 0U;
      } else if (seen) {
        quiet++;
      }
    }

    std::vector<uint16_t> out = hooks::g_dacOut;

    /* Trim most of the dead air off the end, but leave enough for the receiver
       to finish the packet it is in the middle of. */
    size_t last = out.size();
    while (last > 0U && out[last - 1U] == MID)
      last--;

    const size_t keep = last + 480U;
    if (keep < out.size())
      out.resize(keep);

    /* Leave the transmitter idle for whatever runs next in this process. */
    m_tx     = false;
    m_duplex = (DUPLEX == 1);
    hooks::g_dacOut.clear();

    return out;
  }

  std::vector<uint16_t> applyChannel(const std::vector<uint16_t>& in, const Channel& ch)
  {
    std::vector<uint16_t> out;
    out.reserve(in.size() + ch.phase);

    for (unsigned i = 0U; i < ch.phase; i++)
      out.push_back(MID);

    for (size_t i = 0U; i < in.size(); i++) {
      double v = double(int(in[i]) - int(MID));

      if (ch.invert)
        v = -v;

      v *= ch.gain;

      int s = int(std::floor(double(MID) + v + 0.5)) + ch.dcOffset;

      if (s < 0)
        s = 0;
      if (s > 4095)
        s = 4095;

      out.push_back(uint16_t(s));
    }

    return out;
  }

  std::vector<uint16_t> silence(size_t samples)
  {
    return std::vector<uint16_t>(samples, MID);
  }

  void append(std::vector<uint16_t>& dst, const std::vector<uint16_t>& src)
  {
    dst.insert(dst.end(), src.begin(), src.end());
  }

  std::vector<std::vector<uint8_t> > demodulate(const std::vector<uint16_t>& adc, bool resetReceiver)
  {
    m_mode = 2U;
    m_tx   = false;

    if (resetReceiver)
      mode2RX.reset();

    hooks::g_kissTx.clear();
    hooks::g_debugTx.clear();
    hooks::g_dacOut.clear();
    hooks::g_adcIn    = adc;
    hooks::g_adcInPtr = 0U;

    /* Enough extra ticks for the receiver to run out the last packet. */
    const size_t ticks = adc.size() + 8192U;

    for (size_t t = 0U; t < ticks; t++) {
      io.interrupt();

      /* On target the main loop runs orders of magnitude faster than the
         24 kHz sample clock. Three passes per tick keeps the RX ring buffer
         drained, which is what the hardware sees. */
      io.process();
      io.process();
      io.process();

      /* Keep the DAC capture from growing without bound over long runs. */
      if (hooks::g_dacOut.size() > 65536U)
        hooks::g_dacOut.clear();
    }

    return hooks::kissFrames();
  }

  std::vector<std::vector<uint8_t> > loopback(const std::vector<uint8_t>& payload, const Channel& ch)
  {
    const std::vector<uint16_t> clean = modulate(payload.empty() ? NULL : &payload[0], uint16_t(payload.size()));
    const std::vector<uint16_t> dirty = applyChannel(clean, ch);

    return demodulate(dirty);
  }

  bool decodedExactly(const std::vector<std::vector<uint8_t> >& frames, const std::vector<uint8_t>& payload)
  {
    if (frames.size() != 1U)
      return false;

    const std::vector<uint8_t>& f = frames[0];
    if (f.size() != payload.size() + 1U)
      return false;

    for (size_t i = 0U; i < payload.size(); i++) {
      if (f[i + 1U] != payload[i])
        return false;
    }

    return true;
  }

}

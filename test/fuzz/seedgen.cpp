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
 * Generates the seed corpora, using the firmware itself so every seed is a
 * genuine artefact: real KISS frames, a real IL2P encoding, real modulated
 * bursts. Coverage guided mutation of valid inputs reaches the deep decode
 * paths far sooner than mutation of noise.
 *
 * Usage: seedgen <corpus directory>
 */

#include "Config.h"
#include "Globals.h"
#include "KISSDefines.h"

#include "IL2PTX.h"

#include "shim/TestHooks.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

  std::vector<uint8_t> samplePayload(size_t length)
  {
    std::vector<uint8_t> p;
    uint8_t x = 0x5AU;
    for (size_t i = 0U; i < length; i++) {
      x = uint8_t(x * 31U + 17U);
      p.push_back(i == 13U ? uint8_t(x & 0xFEU) : x);
    }
    return p;
  }

  std::vector<uint8_t> sampleUIFrame()
  {
    std::vector<uint8_t> f;
    const char* dest = "TEST  ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(dest[i]) << 1);
    f.push_back(0xE0U);
    const char* src = "M0LTE ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(src[i]) << 1);
    f.push_back(0x62U | 0x01U);
    f.push_back(0x03U);
    f.push_back(0xF0U);
    const char* info = "FUZZ SEED";
    for (const char* p = info; *p != '\0'; p++)
      f.push_back(uint8_t(*p));
    return f;
  }

  std::vector<uint8_t> kissEncode(uint8_t type, const std::vector<uint8_t>& content)
  {
    std::vector<uint8_t> out;
    out.push_back(KISS_FEND);
    out.push_back(type);
    for (size_t i = 0U; i < content.size(); i++) {
      const uint8_t c = content[i];
      if (c == KISS_FEND) {
        out.push_back(KISS_FESC);
        out.push_back(KISS_TFEND);
      } else if (c == KISS_FESC) {
        out.push_back(KISS_FESC);
        out.push_back(KISS_TFESC);
      } else {
        out.push_back(c);
      }
    }
    out.push_back(KISS_FEND);
    return out;
  }

  void writeSeed(const std::string& path, const std::vector<uint8_t>& bytes)
  {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == NULL) {
      std::fprintf(stderr, "cannot write %s\n", path.c_str());
      std::exit(1);
    }
    std::fwrite(&bytes[0], 1U, bytes.size(), f);
    std::fclose(f);
    std::printf("  %-40s %5zu bytes\n", path.c_str(), bytes.size());
  }

  /* Pump the selected transmitter until the DAC goes quiet, and return its
     output in the fuzzers' sample format: little-endian q15 pairs, scaled
     the way CIO::process() scales the ADC. */
  std::vector<uint8_t> pumpTX(uint8_t mode)
  {
    m_mode   = mode;
    m_duplex = true;
    m_tx     = false;

    hooks::g_dacOut.clear();

    unsigned quiet = 0U;
    bool     seen  = false;

    for (unsigned t = 0U; t < 24000U * 6U && quiet < 480U; t++) {
      if (mode == 1U)
        ax25TX.process();
      else
        mode2TX.process();
      io.interrupt();

      const uint16_t s = hooks::g_dacOut.back();
      if (s != 2048U) {
        seen  = true;
        quiet = 0U;
      } else if (seen) {
        quiet++;
      }
    }

    std::vector<uint8_t> out;
    for (size_t i = 0U; i < hooks::g_dacOut.size(); i++) {
      const int32_t q = ((int32_t(hooks::g_dacOut[i]) - 2048) * 16384) >> 15;
      out.push_back(uint8_t(q & 0xFF));
      out.push_back(uint8_t((q >> 8) & 0xFF));
    }

    /* One process() call so CIO drops the PTT it raised. */
    io.process();
    m_tx = false;

    return out;
  }

}

int main(int argc, char** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: seedgen <corpus directory>\n");
    return 1;
  }

  const std::string base = argv[1];

  const std::vector<uint8_t> payload = samplePayload(48U);
  const std::vector<uint8_t> ui      = sampleUIFrame();

  /* ---- kiss: well formed traffic and every command --------------------- */

  writeSeed(base + "/kiss/data", kissEncode(KISS_TYPE_DATA, payload));
  writeSeed(base + "/kiss/data_ui", kissEncode(KISS_TYPE_DATA, ui));

  std::vector<uint8_t> reserved = payload;
  reserved[0] = KISS_FEND;
  reserved[1] = KISS_FESC;
  reserved[2] = KISS_TFEND;
  reserved[3] = KISS_TFESC;
  writeSeed(base + "/kiss/data_escaped", kissEncode(KISS_TYPE_DATA, reserved));

  std::vector<uint8_t> ack;
  ack.push_back(0x34U);
  ack.push_back(0x12U);
  ack.insert(ack.end(), payload.begin(), payload.end());
  writeSeed(base + "/kiss/data_with_ack", kissEncode(KISS_TYPE_DATA_WITH_ACK, ack));

  writeSeed(base + "/kiss/tx_delay", kissEncode(KISS_TYPE_TX_DELAY, std::vector<uint8_t>(1U, 30U)));
  writeSeed(base + "/kiss/p_persistence", kissEncode(KISS_TYPE_P_PERSISTENCE, std::vector<uint8_t>(1U, 63U)));
  writeSeed(base + "/kiss/slot_time", kissEncode(KISS_TYPE_SLOT_TIME, std::vector<uint8_t>(1U, 10U)));
  writeSeed(base + "/kiss/tx_tail", kissEncode(KISS_TYPE_TX_TAIL, std::vector<uint8_t>(1U, 5U)));
  writeSeed(base + "/kiss/full_duplex", kissEncode(KISS_TYPE_FULL_DUPLEX, std::vector<uint8_t>(1U, 1U)));
  writeSeed(base + "/kiss/set_mode", kissEncode(KISS_TYPE_SET_HARDWARE, std::vector<uint8_t>(1U, 3U)));

  std::vector<uint8_t> levels;
  levels.push_back(128U);
  levels.push_back(128U);
  levels.push_back(128U);
  levels.push_back(128U);
  writeSeed(base + "/kiss/set_levels", kissEncode(KISS_TYPE_SET_HARDWARE, levels));

  /* ---- il2p_tx: payloads for the encoder ------------------------------- */

  writeSeed(base + "/il2p_tx/raw", payload);
  writeSeed(base + "/il2p_tx/ui_frame", ui);
  writeSeed(base + "/il2p_tx/large", samplePayload(600U));

  /* ---- il2p_rx: real encodings for the decoder ------------------------- */

  {
    CIL2PTX tx;
    uint8_t encoded[IL2P_MAX_ENCODED_LENGTH];

    uint16_t len = tx.process(&payload[0], uint16_t(payload.size()), encoded);
    writeSeed(base + "/il2p_rx/raw", std::vector<uint8_t>(encoded, encoded + len));

    len = tx.process(&ui[0], uint16_t(ui.size()), encoded);
    writeSeed(base + "/il2p_rx/ui_frame", std::vector<uint8_t>(encoded, encoded + len));
  }

  /* ---- mode2_rx / ax25_rx: real bursts, in q15 sample pairs ------------ */

  std::printf("modulating the mode 2 seed burst...\n");
  mode2TX.setTXDelay(3U);
  if (mode2TX.writeData(&payload[0], uint16_t(payload.size())) != 0U) {
    std::fprintf(stderr, "mode 2 writeData failed\n");
    return 1;
  }
  writeSeed(base + "/mode2_rx/burst", pumpTX(2U));

  std::printf("modulating the AX.25 seed burst...\n");
  ax25TX.setTXDelay(30U);
  ax25TX.writeData(&ui[0], uint16_t(ui.size()));
  writeSeed(base + "/ax25_rx/burst", pumpTX(1U));

  return 0;
}

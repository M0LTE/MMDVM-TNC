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
 * Tests that establish the harness is wired up correctly and that the layers
 * either side of the modem work. If any of these fail, nothing else in the
 * suite means anything.
 *
 * Everything here is deliberately independent of the mode 2 receiver, so this
 * file stays green regardless of the state of Mode2RX.cpp.
 */

#include "Config.h"
#include "Globals.h"

#include "AX25CRC.h"
#include "Hamming.h"
#include "IL2PRS.h"
#include "IL2PRX.h"
#include "IL2PTX.h"
#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

/* ------------------------------------------------------- forward error --- */

TF_TEST(baseline_hamming_round_trip)
{
  CHamming hamming;

  for (uint8_t n = 0U; n < 16U; n++) {
    const uint8_t encoded = hamming.encode(n);
    CHECK_EQ(int(hamming.decode(encoded)), int(n));

    /* Every single bit error must be corrected. */
    for (uint8_t b = 0U; b < 8U; b++) {
      const uint8_t corrupt = uint8_t(encoded ^ (1U << b));
      CHECK_MSG(hamming.decode(corrupt) == n, "nibble " << int(n) << " bit " << int(b));
    }
  }
}

TF_TEST(baseline_crc_is_stable)
{
  CAX25CRC crc;

  const uint8_t a[] = { 0x01U, 0x02U, 0x03U, 0x04U };
  const uint8_t b[] = { 0x01U, 0x02U, 0x03U, 0x05U };

  const uint16_t ca = crc.calculate(a, 4U);

  CHECK_EQ(int(crc.calculate(a, 4U)), int(ca));
  CHECK(crc.calculate(b, 4U) != ca);
}

TF_TEST(baseline_reed_solomon_corrects_within_capacity)
{
  /* RS(255, 239): eight symbol errors are correctable. */
  CIL2PRS rs(16U);

  uint8_t block[255];
  for (int i = 0; i < 239; i++)
    block[i] = uint8_t(i * 7 + 3);

  uint8_t parity[16];
  rs.encode(block, parity);
  ::memcpy(block + 239, parity, 16U);

  uint8_t locs[16];
  CHECK_EQ(rs.decode(block, locs), 0);

  uint8_t expected[255];
  ::memcpy(expected, block, 255U);

  block[10]  ^= 0xFFU;
  block[80]  ^= 0x5AU;
  block[200] ^= 0x01U;

  CHECK_EQ(rs.decode(block, locs), 3);
  CHECK(::memcmp(block, expected, 255U) == 0);
}

TF_TEST(baseline_reed_solomon_reports_uncorrectable_as_negative)
{
  /* The decoder's contract on failure is a negative count. Everything that
     calls it has to honour that. */
  CIL2PRS rs(2U);                      /* RS(255, 253): one correctable symbol */

  uint8_t block[255];
  for (int i = 0; i < 253; i++)
    block[i] = uint8_t(i * 11 + 5);

  uint8_t parity[2];
  rs.encode(block, parity);
  ::memcpy(block + 253, parity, 2U);

  uint8_t locs[16];
  REQUIRE_EQ(rs.decode(block, locs), 0);

  block[3]   ^= 0xFFU;
  block[44]  ^= 0xFFU;
  block[150] ^= 0xFFU;
  block[201] ^= 0xFFU;

  CHECK_MSG(rs.decode(block, locs) < 0, "an uncorrectable block must not report success");
}

/* ------------------------------------------------------- IL2P framing ---- */

TF_TEST(baseline_il2p_raw_payload_round_trips)
{
  const std::vector<uint8_t> payload = radio::rawPayload(64U);

  CIL2PTX tx;
  uint8_t encoded[2000U];
  const uint16_t len = tx.process(&payload[0], uint16_t(payload.size()), encoded);
  REQUIRE(len > 0U);

  CIL2PRX rx;
  uint8_t out[1100U];

  REQUIRE(rx.processHeader(encoded, out));
  REQUIRE_EQ(int(rx.getPayloadLength()), int(payload.size()));

  const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;
  REQUIRE(rx.processPayload(encoded + hdrBytes, out));

  const uint16_t parity = rx.getPayloadParityLength();
  REQUIRE(rx.checkCRC(out, encoded + hdrBytes + rx.getPayloadLength() + parity));

  REQUIRE_EQ(int(rx.getHeaderLength() + rx.getPayloadLength()), int(payload.size()));
  CHECK(::memcmp(out, &payload[0], payload.size()) == 0);
}

TF_TEST(baseline_il2p_ax25_ui_frame_round_trips)
{
  /* A real AX.25 UI frame takes the type 1 (translated) header path, where the
     addresses are rebuilt from IL2P fields rather than carried verbatim. */
  const std::vector<uint8_t> frame = radio::sampleUIFrame("HELLO WORLD");

  CIL2PTX tx;
  uint8_t encoded[2000U];
  const uint16_t len = tx.process(&frame[0], uint16_t(frame.size()), encoded);
  REQUIRE(len > 0U);

  CIL2PRX rx;
  uint8_t out[1100U];

  REQUIRE(rx.processHeader(encoded, out));

  const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;
  REQUIRE(rx.processPayload(encoded + hdrBytes, out));

  const uint16_t total = rx.getHeaderLength() + rx.getPayloadLength();
  REQUIRE_EQ(int(total), int(frame.size()));

  CHECK(::memcmp(out, &frame[0], 6U) == 0);          /* destination callsign */
  CHECK(::memcmp(out + 7, &frame[7], 6U) == 0);      /* source callsign */
  CHECK_EQ(int(out[14]), int(0x03U));                /* UI */
  CHECK_EQ(int(out[15]), int(0xF0U));                /* no layer 3 */
  CHECK(::memcmp(out + 16, &frame[16], frame.size() - 16U) == 0);
}

/* ------------------------------------------------------- the wire format -- */

TF_TEST(baseline_sync_vector_symbol_mapping)
{
  /*
   * Pins down the relationship the receiver depends on, and which several of
   * the tests that follow rely on.
   *
   * Mode2TX maps dibits to levels 11 -> +3, 10 -> +1, 00 -> -1, 01 -> -3.
   * MODE2_SYNC_BYTES therefore goes on the air as the NEGATIVE of
   * MODE2_SYNC_SYMBOLS_VALUES, and the sign bits of the transmitted symbols
   * spell out MODE2_SYNC_SYMBOLS.
   */
  std::vector<uint8_t> syncBytes(MODE2_SYNC_BYTES, MODE2_SYNC_BYTES + MODE2_SYNC_LENGTH_BYTES);
  const std::vector<int8_t> syms = radio::bytesToSymbols(syncBytes);

  REQUIRE_EQ(int(syms.size()), int(MODE2_SYNC_LENGTH_SYMBOLS));

  uint16_t signBits = 0U;

  for (unsigned i = 0U; i < MODE2_SYNC_LENGTH_SYMBOLS; i++) {
    CHECK_MSG(syms[i] == -MODE2_SYNC_SYMBOLS_VALUES[i],
              "sync symbol " << i << " is " << int(syms[i])
              << ", expected " << int(-MODE2_SYNC_SYMBOLS_VALUES[i]));

    signBits = uint16_t(signBits << 1);
    if (syms[i] < 0)
      signBits = uint16_t(signBits | 0x0001U);
  }

  CHECK_MSG(signBits == MODE2_SYNC_SYMBOLS,
            "sign pattern 0x" << std::hex << signBits
            << " != MODE2_SYNC_SYMBOLS 0x" << MODE2_SYNC_SYMBOLS);

  /* Only the top bit of each dibit sets the sign, which is what lets a test
     inject magnitude-only errors that leave the sign correlator untouched. */
  for (unsigned d = 0U; d < 4U; d++) {
    std::vector<uint8_t> one(1U, uint8_t(d << 6));
    const std::vector<int8_t> s = radio::bytesToSymbols(one);
    const bool negative = s[0] < 0;
    CHECK_EQ(negative, (d & 0x02U) == 0U);
  }
}

TF_TEST(baseline_firmware_transmitter_produces_a_burst)
{
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  const std::vector<uint16_t> wave = radio::modulate(&payload[0], uint16_t(payload.size()));

  REQUIRE_MSG(wave.size() > 1000U, "only got " << wave.size() << " samples");

  uint16_t lo = 4095U;
  uint16_t hi = 0U;
  for (size_t i = 0U; i < wave.size(); i++) {
    if (wave[i] < lo) lo = wave[i];
    if (wave[i] > hi) hi = wave[i];
  }

  CHECK_MSG(hi > radio::MID + 200U, "peak only " << hi);
  CHECK_MSG(lo < radio::MID - 200U, "trough only " << lo);
}

TF_TEST(baseline_reference_modulator_produces_a_matching_burst)
{
  /* The reference modulator exists so tests can put deliberately wrong
     symbols on the air. It has to produce the same shape of signal as the
     firmware transmitter, at the same level. */
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  const std::vector<uint16_t> fw  = radio::modulate(&payload[0], uint16_t(payload.size()));
  const std::vector<uint16_t> ref = radio::modulateSymbols(radio::bytesToSymbols(radio::onAirBytes(payload)));

  REQUIRE(fw.size() > 1000U);
  REQUIRE(ref.size() > 1000U);

  int fwPeak  = 0;
  int refPeak = 0;
  for (size_t i = 0U; i < fw.size(); i++)
    fwPeak = std::max(fwPeak, std::abs(int(fw[i]) - int(radio::MID)));
  for (size_t i = 0U; i < ref.size(); i++)
    refPeak = std::max(refPeak, std::abs(int(ref[i]) - int(radio::MID)));

  CHECK_MSG(std::abs(fwPeak - refPeak) <= 8,
            "peak deviation differs: firmware " << fwPeak << ", reference " << refPeak);
}

/* ------------------------------------------------------- the harness ----- */

TF_TEST(baseline_fir_shim_honours_the_cmsis_contract)
{
  /* A sanity check on the shim itself: a known impulse response. */
  armshim::resetCounters();

  q15_t coeffs[4] = { 1000, 2000, 4000, 8000 };
  q15_t state[8]  = { 0 };

  arm_fir_instance_q15 f;
  f.numTaps = 4U;
  f.pState  = state;
  f.pCoeffs = coeffs;

  q15_t in[4]  = { 32767, 0, 0, 0 };
  q15_t out[4] = { 0 };

  ::arm_fir_fast_q15(&f, in, out, 4U);

  /* CMSIS stores coefficients time reversed, so an impulse walks the array
     backwards. */
  CHECK_EQ(int(out[0]), 7999);
  CHECK_EQ(int(out[1]), 3999);
  CHECK_EQ(int(out[2]), 1999);
  CHECK_EQ(int(out[3]),  999);

  CHECK_EQ(int(armshim::g_oddTapCalls), 0);
  CHECK_EQ(int(armshim::g_coeffOverrunReads), 0);
}

TF_TEST(baseline_each_test_gets_clean_globals)
{
  /* The suite forks per test, so state cannot leak between them. If this ever
     fails, every other result in the suite is suspect. */
  CHECK_EQ(int(hooks::g_ticks), 0);
  CHECK_EQ(int(hooks::g_kissTx.size()), 0);
  CHECK_EQ(int(hooks::g_debugTx.size()), 0);
  CHECK_EQ(int(m_mode), int(INITIAL_MODE));
  CHECK_EQ(m_tx, false);
}

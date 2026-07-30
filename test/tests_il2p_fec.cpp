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
 * What the IL2P layer does when forward error correction gives up.
 *
 * CIL2PRS::decode() returns a NEGATIVE count for an uncorrectable block and
 * leaves the data untouched. baseline_reed_solomon_reports_uncorrectable_as
 * _negative covers that contract directly; everything above it has to honour
 * the sign.
 *
 * Two different failure modes are easy to confuse, so to be clear about which
 * of them these tests cover:
 *
 *  - Uncorrectable, reported as such. The decoder gives up and returns -1.
 *    Reachable on the payload, which carries 16 parity symbols.
 *
 *  - Miscorrection. The decoder finds a plausible but wrong error pattern and
 *    returns a positive count. This is what actually happens to a badly
 *    damaged header: with only two parity symbols the error locator
 *    polynomial is degree one, which always has a root, so -1 never comes
 *    back. Measured over 3000 randomly damaged headers, RS(255,253) returned
 *    -1 zero times and miscorrected every time. CIL2PRX::decode already
 *    guards against most of that by rejecting any correction that lands in
 *    the zero padding, which caught about 90% of them.
 *
 * These tests are about the first case.
 */

#include "Config.h"
#include "Globals.h"

#include "IL2PRX.h"
#include "IL2PTX.h"
#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <cstring>

namespace {

  const unsigned PREAMBLE_BYTES = 36U;
  const unsigned HEADER_BYTES   = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;

}

TF_TEST(il2p_rejects_a_header_beyond_the_correcting_power_of_its_parity)
{
  /*
   * The IL2P header is 13 bytes plus 2 parity symbols. RS(255,253) corrects
   * one byte error, so three must be refused. Nothing downstream re-checks
   * the header, and its ten payload-length bits decide how long Mode2RX
   * spends in MODE2RXS_PAYLOAD, where correlateSync() is not running.
   *
   * Green today: the decoder miscorrects rather than returning -1, and the
   * padding guard in CIL2PRX::decode catches this particular pattern. Here as
   * a regression guard on that path.
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  CIL2PTX tx;
  uint8_t encoded[2000U];
  REQUIRE(tx.process(&payload[0], uint16_t(payload.size()), encoded) > 0U);

  uint8_t corrupt[2000U];
  ::memcpy(corrupt, encoded, 2000U);
  corrupt[1] ^= 0xFFU;
  corrupt[5] ^= 0xFFU;
  corrupt[9] ^= 0xFFU;

  CIL2PRX rx;
  uint8_t out[1100U];

  CHECK_MSG(!rx.processHeader(corrupt, out),
            "a header with three byte errors was accepted, "
            "but its two parity symbols can only correct one");
}

TF_TEST(il2p_accepts_a_header_within_the_correcting_power_of_its_parity)
{
  /* The other half of the contract, so the fix cannot be "always fail". */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  CIL2PTX tx;
  uint8_t encoded[2000U];
  REQUIRE(tx.process(&payload[0], uint16_t(payload.size()), encoded) > 0U);

  uint8_t corrupt[2000U];
  ::memcpy(corrupt, encoded, 2000U);
  corrupt[4] ^= 0xFFU;                 /* one byte error: correctable */

  CIL2PRX rx;
  uint8_t out[1100U];

  REQUIRE_MSG(rx.processHeader(corrupt, out),
              "a header with one correctable byte error was rejected");
  CHECK_EQ(int(rx.getPayloadLength()), int(payload.size()));
}

TF_TEST(il2p_rejects_a_payload_block_beyond_its_correcting_power)
{
  /* The payload carries 16 parity symbols, so eight byte errors are
     correctable and more are not -- and with that many roots the decoder
     really does return -1 rather than miscorrecting. This is the case
     CIL2PRX::decode gets wrong. */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  CIL2PTX tx;
  uint8_t encoded[2000U];
  REQUIRE(tx.process(&payload[0], uint16_t(payload.size()), encoded) > 0U);

  CIL2PRX rx;
  uint8_t out[1100U];
  REQUIRE(rx.processHeader(encoded, out));

  uint8_t corrupt[2000U];
  ::memcpy(corrupt, encoded, 2000U);
  for (unsigned i = 0U; i < 12U; i++)
    corrupt[HEADER_BYTES + i * 2U] ^= 0xFFU;

  CHECK_MSG(!rx.processPayload(corrupt + HEADER_BYTES, out),
            "a payload block with twelve byte errors was accepted, "
            "but its parity can only correct eight");
}

TF_TEST(il2p_accepts_a_payload_block_within_its_correcting_power)
{
  /* The other half, so the fix cannot be "always fail". */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  CIL2PTX tx;
  uint8_t encoded[2000U];
  REQUIRE(tx.process(&payload[0], uint16_t(payload.size()), encoded) > 0U);

  CIL2PRX rx;
  uint8_t out[1100U];
  REQUIRE(rx.processHeader(encoded, out));

  uint8_t corrupt[2000U];
  ::memcpy(corrupt, encoded, 2000U);
  for (unsigned i = 0U; i < 6U; i++)
    corrupt[HEADER_BYTES + i * 3U] ^= 0xFFU;      /* six errors: correctable */

  REQUIRE_MSG(rx.processPayload(corrupt + HEADER_BYTES, out),
              "a payload block with six byte errors was rejected, "
              "but its parity can correct eight");
  CHECK(::memcmp(out, &payload[0], payload.size()) == 0);
}

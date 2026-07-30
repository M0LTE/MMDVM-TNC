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
 * IL2P payload sizes. The framer splits the payload into Reed-Solomon blocks
 * whose count and sizes change at fixed thresholds, and the block boundary
 * arithmetic is exactly where an off by one would live. The rest of the suite
 * only ever carries mid sized payloads, so every boundary is walked here,
 * plus the largest frame IL2P can label, end to end over the air.
 */

#include "Config.h"
#include "Globals.h"

#include "IL2PRX.h"
#include "IL2PTX.h"
#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"

#include <cstring>

namespace {

  void roundTrip(uint16_t n)
  {
    const std::vector<uint8_t> payload = radio::rawPayload(n);

    CIL2PTX tx;
    uint8_t encoded[2000U];
    const uint16_t len = tx.process(&payload[0], n, encoded);
    REQUIRE_MSG(len > 0U, "nothing encoded at length " << n);

    CIL2PRX rx;
    uint8_t out[1100U];

    REQUIRE_MSG(rx.processHeader(encoded, out), "header rejected at length " << n);
    REQUIRE_MSG(rx.getPayloadLength() == n,
                "header says " << rx.getPayloadLength() << " bytes, sent " << n);

    const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;
    REQUIRE_MSG(rx.processPayload(encoded + hdrBytes, out), "payload rejected at length " << n);

    const uint16_t parity = rx.getPayloadParityLength();
    REQUIRE_MSG(rx.checkCRC(out, encoded + hdrBytes + rx.getPayloadLength() + parity),
                "CRC failed at length " << n);

    CHECK_MSG(::memcmp(out, &payload[0], n) == 0, "payload corrupted at length " << n);
  }

}

TF_TEST(il2p_round_trips_across_the_block_boundaries)
{
  /* 239 is the most one RS(255) block can carry, so the interesting edges sit
     either side of each multiple of it, plus the ends of the range. */
  const uint16_t sizes[] = { 1U, 2U, 100U, 238U, 239U, 240U,
                             477U, 478U, 479U, 1000U, 1023U };

  for (size_t i = 0U; i < sizeof(sizes) / sizeof(sizes[0]); i++)
    roundTrip(sizes[i]);
}

TF_TEST(mode2_carries_the_largest_il2p_frame_over_the_air)
{
  /* 1023 bytes is the biggest payload the IL2P header can label. It also
     comes closest to filling the transmitter's byte FIFO and the receiver's
     working buffers, which no other test approaches. */
  const std::vector<uint8_t> payload = radio::rawPayload(1023U);

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::loopback(payload, ch), payload));
}

TF_TEST(transmitters_reject_a_payload_too_long_for_il2p)
{
  /* The header's byte count is a ten bit field. A 1024 byte payload would go
     on the air labelled as 0 bytes, so it has to be refused at the door --
     and quietly overflowing the encode buffer would be worse still. */
  const std::vector<uint8_t> tooLong = radio::rawPayload(1024U);

  CHECK_EQ(int(mode2TX.writeData(&tooLong[0], 1024U)), 4);
  CHECK_EQ(int(mode3TX.writeData(&tooLong[0], 1024U)), 4);

  /* The largest legal payload is still accepted. */
  CHECK_EQ(int(mode2TX.writeData(&tooLong[0], 1023U)), 0);
  CHECK_EQ(int(mode3TX.writeData(&tooLong[0], 1023U)), 0);
}

TF_TEST(mode2_maximum_tx_delay_carries_the_largest_frame)
{
  /* TX delay 255 is 3060 bytes of preamble before the frame even starts;
     with the largest payload behind it the whole burst has to fit in the
     transmitter's FIFO, which used to hold only 3000 bytes and silently tore
     the frame. */
  const std::vector<uint8_t> payload = radio::rawPayload(1023U);

  const std::vector<uint16_t> wave = radio::modulate(&payload[0], 1023U, 255U);
  REQUIRE_MSG(wave.size() > 80000U, "burst is only " << wave.size() << " samples");

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(mode3_maximum_tx_delay_and_largest_frame_fit_the_fifo)
{
  const std::vector<uint8_t> payload = radio::rawPayload(1023U);

  mode3TX.setTXDelay(255U);
  CHECK_EQ(int(mode3TX.writeData(&payload[0], 1023U)), 0);
}

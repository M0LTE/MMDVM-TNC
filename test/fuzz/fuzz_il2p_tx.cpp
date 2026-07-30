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
 * Fuzz the IL2P encoder with arbitrary payloads, and hold it to a property:
 * whatever it encodes, the decoder must get back byte for byte. The type 1
 * header path rebuilds AX.25 addresses from translated fields, the type 0
 * path carries bytes verbatim, and the fuzzer's job is to find a payload
 * where either encoder writes out of bounds or the round trip lies.
 *
 * Byte-exactness is not a nicety: the receiver computes the frame CRC over
 * the REBUILT bytes, so any frame isIL2PType1() accepts but the header
 * cannot represent exactly is thrown away by the far end's CRC check.
 */

#include "Config.h"

#include "IL2PRX.h"
#include "IL2PTX.h"
#include "Mode2Defines.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  if (size > IL2P_MAX_PAYLOAD_LENGTH)
    size = IL2P_MAX_PAYLOAD_LENGTH;

  CIL2PTX tx;
  uint8_t encoded[IL2P_MAX_ENCODED_LENGTH];
  const uint16_t len = tx.process(data, uint16_t(size), encoded);

  if (len == 0U || len > IL2P_MAX_ENCODED_LENGTH) {
    std::fprintf(stderr, "encoder produced %u bytes from a %zu byte payload\n", len, size);
    std::abort();
  }

  CIL2PRX rx;
  uint8_t out[1100U];

  if (!rx.processHeader(encoded, out)) {
    std::fprintf(stderr, "decoder rejected a clean header, payload %zu bytes\n", size);
    std::abort();
  }

  const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;

  if (rx.getPayloadLength() > 0U) {
    if (!rx.processPayload(encoded + hdrBytes, out)) {
      std::fprintf(stderr, "decoder rejected a clean payload, %zu bytes\n", size);
      std::abort();
    }
  }

  const uint16_t parity = rx.getPayloadParityLength();
  if (!rx.checkCRC(out, encoded + hdrBytes + rx.getPayloadLength() + parity)) {
    std::fprintf(stderr, "clean round trip failed the CRC, payload %zu bytes\n", size);
    std::abort();
  }

  if (size_t(rx.getHeaderLength()) + size_t(rx.getPayloadLength()) != size) {
    std::fprintf(stderr, "length changed in transit: sent %zu, got %u + %u\n",
                 size, rx.getHeaderLength(), rx.getPayloadLength());
    std::abort();
  }

  for (size_t i = 0U; i < size; i++) {
    if (out[i] != data[i]) {
      std::fprintf(stderr, "byte %zu changed in transit: sent %02x, got %02x\n",
                   i, data[i], out[i]);
      std::abort();
    }
  }

  return 0;
}

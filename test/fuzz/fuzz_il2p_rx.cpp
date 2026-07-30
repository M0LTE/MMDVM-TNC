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
 * Fuzz the IL2P decoder with arbitrary frame bytes: descrambling,
 * Reed-Solomon decoding, header parsing and the payload block arithmetic.
 * This is the RF attack surface -- on the air these bytes come from whatever
 * the demodulator sliced out of the noise, so nothing about them can be
 * trusted.
 *
 * The call sequence mirrors CMode2RX exactly: processHeader() first, then
 * processPayload() and checkCRC() on the bytes that follow, all out of a
 * fixed buffer the way the receiver's own sample buffer works.
 */

#include "Config.h"

#include "IL2PRX.h"
#include "Mode2Defines.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  /* The receiver always reads whole fields -- header, payload blocks, CRC --
     from its own fixed-size buffer, so zero-pad the input up to one. */
  static uint8_t frame[2000U];
  ::memset(frame, 0x00U, sizeof(frame));

  const size_t n = size < sizeof(frame) ? size : sizeof(frame);
  ::memcpy(frame, data, n);

  CIL2PRX rx;
  uint8_t out[1100U];

  if (!rx.processHeader(frame, out))
    return 0;

  const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;

  if (rx.getPayloadLength() > 0U) {
    if (!rx.processPayload(frame + hdrBytes, out))
      return 0;
  }

  const uint16_t parity = rx.getPayloadParityLength();
  rx.checkCRC(out, frame + hdrBytes + rx.getPayloadLength() + parity);

  return 0;
}

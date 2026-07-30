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
 * The IL2P type 1 (translated) header, frame type by frame type. Connected
 * mode AX.25 lives or dies on these translations: the receiver rebuilds the
 * frame from the header fields and judges the result against the frame's
 * CRC, so a translation that is not byte for byte exact does not just
 * corrupt a field -- it makes the far end throw the frame away entirely.
 *
 * The bit layout asserted here matches the Dire Wolf IL2P implementation
 * (il2p_header.c), which is the de facto reference: P/F in header control
 * bit 6, N(R) in bits 5-3, C or N(S) in bits 2-0 depending on type.
 */

#include "Config.h"
#include "Globals.h"

#include "IL2PRX.h"
#include "IL2PTX.h"
#include "Mode2Defines.h"

#include "framework.h"

#include <cstring>
#include <iomanip>

namespace {

  /* TEST-0 > M0LTE-1, with the command/response sense in the SSID C bits. */
  std::vector<uint8_t> frameOf(bool command, uint8_t control)
  {
    std::vector<uint8_t> f;

    const char* dest = "TEST  ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(dest[i]) << 1);
    f.push_back(uint8_t((command ? 0x80U : 0x00U) | 0x60U));

    const char* src = "M0LTE ";
    for (int i = 0; i < 6; i++)
      f.push_back(uint8_t(src[i]) << 1);
    f.push_back(uint8_t((command ? 0x00U : 0x80U) | 0x60U | (1U << 1) | 0x01U));

    f.push_back(control);

    return f;
  }

  void append(std::vector<uint8_t>& f, const char* text)
  {
    for (const char* p = text; *p != '\0'; p++)
      f.push_back(uint8_t(*p));
  }

  /* Encode, decode, and require the result byte for byte identical. Returns
     the header length so a caller can also assert which path was taken:
     0 means type 0 (verbatim), anything else is a translated header. */
  uint16_t roundTrip(const std::vector<uint8_t>& frame, const char* what)
  {
    CIL2PTX tx;
    uint8_t encoded[IL2P_MAX_ENCODED_LENGTH];
    const uint16_t len = tx.process(&frame[0], uint16_t(frame.size()), encoded);
    REQUIRE_MSG(len > 0U, what << ": nothing encoded");

    CIL2PRX rx;
    uint8_t out[1100U];

    REQUIRE_MSG(rx.processHeader(encoded, out), what << ": header rejected");

    const uint16_t hdrBytes = MODE2_HEADER_LENGTH_BYTES + MODE2_HEADER_PARITY_BYTES;
    if (rx.getPayloadLength() > 0U)
      REQUIRE_MSG(rx.processPayload(encoded + hdrBytes, out), what << ": payload rejected");

    /* The CRC arbitrates acceptance on the air, so it must pass -- this is
       where a lossy translation turns into a lost frame. */
    const uint16_t parity = rx.getPayloadParityLength();
    REQUIRE_MSG(rx.checkCRC(out, encoded + hdrBytes + rx.getPayloadLength() + parity),
                what << ": a frame the receiver would have thrown away");

    const uint16_t total = rx.getHeaderLength() + rx.getPayloadLength();
    REQUIRE_MSG(total == frame.size(),
                what << ": length changed, sent " << frame.size() << " got " << total);

    for (size_t i = 0U; i < frame.size(); i++)
      CHECK_MSG(out[i] == frame[i],
                what << ": byte " << i << " sent 0x" << std::hex << int(frame[i])
                << " got 0x" << int(out[i]));

    return rx.getHeaderLength();
  }

}

TF_TEST(il2p_s_frames_round_trip_every_sequence_number)
{
  /* RR/RNR/REJ/SREJ x N(R) 0-7 x P/F x command/response: the supervisory
     traffic a connected link exchanges constantly. */
  const uint8_t types[4] = { 0x01U, 0x05U, 0x09U, 0x0DU };

  for (unsigned t = 0U; t < 4U; t++) {
    for (uint8_t nr = 0U; nr < 8U; nr++) {
      for (uint8_t pf = 0U; pf < 2U; pf++) {
        for (uint8_t cmd = 0U; cmd < 2U; cmd++) {
          const uint8_t control = uint8_t(types[t] | (nr << 5) | (pf << 4));
          const uint16_t hdr = roundTrip(frameOf(cmd != 0U, control),
                                         TF_STR("S type 0x" << std::hex << int(types[t])
                                                << std::dec << " N(R) " << int(nr)
                                                << " P/F " << int(pf) << " cmd " << int(cmd)).c_str());
          CHECK_MSG(hdr > 0U, "S frames should take the translated header");
        }
      }
    }
  }
}

TF_TEST(il2p_i_frames_round_trip_every_sequence_number)
{
  for (uint8_t nr = 0U; nr < 8U; nr++) {
    for (uint8_t ns = 0U; ns < 8U; ns++) {
      for (uint8_t pf = 0U; pf < 2U; pf++) {
        const uint8_t control = uint8_t((nr << 5) | (pf << 4) | (ns << 1));

        std::vector<uint8_t> frame = frameOf(true, control);
        frame.push_back(0xF0U);                  /* no layer 3 */
        append(frame, "CONNECTED DATA");

        const uint16_t hdr = roundTrip(frame,
                                       TF_STR("I frame N(R) " << int(nr) << " N(S) " << int(ns)
                                              << " P/F " << int(pf)).c_str());
        CHECK_MSG(hdr > 0U, "I frames should take the translated header");
      }
    }
  }
}

TF_TEST(il2p_u_frames_round_trip)
{
  for (uint8_t pf = 0U; pf < 2U; pf++) {
    const uint8_t p = uint8_t(pf << 4);

    roundTrip(frameOf(true,  uint8_t(0x2FU | p)), "SABM");
    roundTrip(frameOf(true,  uint8_t(0x43U | p)), "DISC");
    roundTrip(frameOf(false, uint8_t(0x0FU | p)), "DM");
    roundTrip(frameOf(false, uint8_t(0x63U | p)), "UA");

    std::vector<uint8_t> frmr = frameOf(false, uint8_t(0x87U | p));
    frmr.push_back(0x01U);                       /* the three FRMR info bytes */
    frmr.push_back(0x2EU);
    frmr.push_back(0x05U);
    roundTrip(frmr, "FRMR");

    for (uint8_t cmd = 0U; cmd < 2U; cmd++) {
      std::vector<uint8_t> xid = frameOf(cmd != 0U, uint8_t(0xAFU | p));
      append(xid, "XID DATA");
      roundTrip(xid, "XID");

      std::vector<uint8_t> test = frameOf(cmd != 0U, uint8_t(0xE3U | p));
      append(test, "TEST DATA");
      roundTrip(test, "TEST");
    }
  }
}

TF_TEST(il2p_ui_frames_round_trip)
{
  for (uint8_t pf = 0U; pf < 2U; pf++) {
    for (uint8_t cmd = 0U; cmd < 2U; cmd++) {
      std::vector<uint8_t> frame = frameOf(cmd != 0U, uint8_t(0x03U | (pf << 4)));
      frame.push_back(0xF0U);
      append(frame, "UI PAYLOAD");

      const uint16_t hdr = roundTrip(frame,
                                     TF_STR("UI P/F " << int(pf) << " cmd " << int(cmd)).c_str());
      CHECK_MSG(hdr > 0U, "UI frames should take the translated header");
    }
  }
}

TF_TEST(il2p_untranslatable_frames_fall_back_to_type_0)
{
  /* Frames the translated header cannot represent exactly must travel
     verbatim instead of being mangled: the SABME opcode, an S frame
     carrying information, an I frame marked as a response, a U frame with
     the wrong command sense, and a stray end-of-address marker. */

  CHECK_EQ(int(roundTrip(frameOf(true, 0x6FU), "SABME")), 0);

  std::vector<uint8_t> rrInfo = frameOf(true, 0x01U);
  append(rrInfo, "X");
  CHECK_EQ(int(roundTrip(rrInfo, "RR with information")), 0);

  std::vector<uint8_t> iResponse = frameOf(false, 0x00U);
  iResponse.push_back(0xF0U);
  append(iResponse, "DATA");
  CHECK_EQ(int(roundTrip(iResponse, "I frame response")), 0);

  CHECK_EQ(int(roundTrip(frameOf(false, 0x2FU), "SABM response")), 0);
  CHECK_EQ(int(roundTrip(frameOf(true, 0x0FU), "DM command")), 0);

  std::vector<uint8_t> marker = frameOf(true, 0x03U);
  marker[2] |= 0x01U;                            /* end-of-address mid-callsign */
  marker.push_back(0xF0U);
  append(marker, "DATA");
  CHECK_EQ(int(roundTrip(marker, "stray address marker")), 0);
}

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
 * Mode 1, the 1200 baud AFSK AX.25 chain: CAX25TX through the virtual radio
 * to CAX25RX, plus the CRC and frame classes underneath them. Until now
 * nothing in the suite touched mode 1 at all.
 */

#include "Config.h"
#include "Globals.h"

#include "AX25CRC.h"
#include "AX25Defines.h"
#include "AX25Frame.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <cstring>

namespace {

  /* The mode 1 receiver keys on real HDLC framing, so give it a generous
     preamble: 30 means 300 ms, the firmware's default TX delay. */
  const uint8_t AX25_TX_DELAY = 30U;

  std::vector<std::vector<uint8_t> > ax25Loopback(const std::vector<uint8_t>& frame,
                                                  const radio::Channel& ch,
                                                  uint8_t txDelay = AX25_TX_DELAY)
  {
    const std::vector<uint16_t> clean = radio::modulate(&frame[0], uint16_t(frame.size()), txDelay, 1U);
    const std::vector<uint16_t> dirty = radio::applyChannel(clean, ch);

    return radio::demodulate(dirty, true, 1U);
  }

}

TF_TEST(ax25_loopback_decodes_a_ui_frame)
{
  const std::vector<uint8_t> frame = radio::sampleUIFrame("THE QUICK BROWN FOX 1200");

  radio::Channel ch;
  CHECK(radio::decodedExactly(ax25Loopback(frame, ch), frame));
}

TF_TEST(ax25_loopback_is_polarity_insensitive)
{
  /* AFSK carries its data in frequencies, so flipping the audio sense must
     not matter at all. */
  const std::vector<uint8_t> frame = radio::sampleUIFrame("UPSIDE DOWN");

  radio::Channel ch;
  ch.invert = true;

  CHECK(radio::decodedExactly(ax25Loopback(frame, ch), frame));
}

TF_TEST(ax25_loopback_survives_bit_stuffing_content)
{
  /* An info field full of flag bytes and long runs of ones leans hard on the
     bit stuffer; any slip shows up as a framing or CRC error. */
  std::vector<uint8_t> frame = radio::sampleUIFrame("");

  const uint8_t nasty[] = { 0x7EU, 0x7EU, 0xFFU, 0xFFU, 0xFFU, 0xFEU,
                            0xC0U, 0xDBU, 0x7DU, 0xAAU, 0x00U, 0xFFU };
  frame.insert(frame.end(), nasty, nasty + sizeof(nasty));

  radio::Channel ch;
  CHECK(radio::decodedExactly(ax25Loopback(frame, ch), frame));
}

TF_TEST(ax25_maximum_tx_delay_with_a_full_frame_stays_in_bounds)
{
  /* The KISS host can ask for a TX delay of 255, i.e. 3060 bits of preamble,
     in front of a maximum length frame. Everything has to fit in the
     transmitter's bit buffer; it used to be 600 bytes, which this overran. */
  const std::vector<uint8_t> frame = radio::rawPayload(AX25_MAX_PACKET_LEN - 2U);

  radio::Channel ch;
  const std::vector<std::vector<uint8_t> > frames = ax25Loopback(frame, ch, 255U);

  CHECK(radio::decodedExactly(frames, frame));
}

TF_TEST(ax25_crc_matches_the_x25_reference)
{
  /* CRC-16/X-25, the AX.25 FCS: the standard's check value for the ASCII
     string "123456789" is 0x906E. Interop depends on this exact polynomial,
     reflection and final XOR. */
  CAX25CRC crc;

  const uint8_t ref[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  CHECK_EQ(int(crc.calculate(ref, 9U)), 0x906E);
}

TF_TEST(ax25_frame_crc_round_trips_and_detects_corruption)
{
  const std::vector<uint8_t> raw = radio::sampleUIFrame("CRC CHECK");

  CAX25Frame frame(&raw[0], uint16_t(raw.size()));
  frame.addCRC();

  CHECK_EQ(int(frame.m_length), int(raw.size() + 2U));
  CHECK(frame.checkCRC());

  frame.m_data[5] ^= 0x01U;
  CHECK(!frame.checkCRC());
}

TF_TEST(ax25_frame_respects_its_capacity)
{
  CAX25Frame frame;

  for (uint16_t i = 0U; i < AX25_MAX_PACKET_LEN; i++)
    CHECK(frame.append(i));

  CHECK(!frame.append(0U));
  CHECK_EQ(int(frame.m_length), int(AX25_MAX_PACKET_LEN));

  /* The data constructor leaves room for the FCS that addCRC() appends. */
  std::vector<uint8_t> big(500U, 0xAAU);
  CAX25Frame truncated(&big[0], uint16_t(big.size()));
  CHECK_EQ(int(truncated.m_length), int(AX25_MAX_PACKET_LEN - 2U));
}

TF_TEST(ax25_transmitter_rejects_an_overlong_frame)
{
  /* CAX25Frame would silently truncate it and then put a VALID CRC on the
     truncated bytes -- undetectable corruption at the far end -- so the
     transmitter must refuse it outright. */
  const std::vector<uint8_t> tooLong = radio::rawPayload(AX25_MAX_PACKET_LEN - 1U);

  CHECK_EQ(int(ax25TX.writeData(&tooLong[0], uint16_t(tooLong.size()))), 4);
  CHECK_EQ(int(ax25TX.writeData(&tooLong[0], AX25_MAX_PACKET_LEN - 2U)), 0);
}

TF_TEST(ax25_frame_handed_over_mid_transmission_is_queued_not_clobbered)
{
  /* writeData() used to reset the bit buffer unconditionally, so a packet
     arriving while another was going out corrupted the one on the air and
     restarted the transmitter mid carrier. It queues behind it now. */
  const std::vector<uint8_t> first  = radio::sampleUIFrame("FIRST FRAME OUT");
  const std::vector<uint8_t> second = radio::sampleUIFrame("SECOND, QUEUED MID BURST");

  m_mode   = 1U;
  m_duplex = true;
  m_tx     = false;

  hooks::g_dacOut.clear();

  ax25TX.setTXDelay(30U);
  REQUIRE_EQ(int(ax25TX.writeData(&first[0], uint16_t(first.size()))), 0);

  /* Run the transmitter into the middle of the first frame... */
  for (unsigned t = 0U; t < 4000U; t++) {
    ax25TX.process();
    io.interrupt();
  }
  REQUIRE(hooks::g_ptt);

  /* ...hand over the second, then run everything out. */
  REQUIRE_EQ(int(ax25TX.writeData(&second[0], uint16_t(second.size()))), 0);

  for (unsigned t = 0U; t < 24000U * 5U; t++) {
    ax25TX.process();
    io.interrupt();
  }

  const std::vector<uint16_t> wave = hooks::g_dacOut;
  hooks::g_dacOut.clear();
  m_tx     = false;
  m_duplex = (DUPLEX == 1);

  radio::Channel ch;
  const std::vector<std::vector<uint8_t> > frames =
    radio::demodulate(radio::applyChannel(wave, ch), true, 1U);

  REQUIRE_EQ(int(frames.size()), 2);
  CHECK(frames[0].size() == first.size() + 1U &&
        ::memcmp(&frames[0][1], &first[0], first.size()) == 0);
  CHECK(frames[1].size() == second.size() + 1U &&
        ::memcmp(&frames[1][1], &second[0], second.size()) == 0);
}

TF_TEST(ax25_full_bit_buffer_rejects_the_new_frame_and_keeps_the_old)
{
  /* With the longest TX delay in front of a maximum length frame there is
     no room to queue a second one. It must be refused whole -- and the
     frame already on its way out must still arrive intact. */
  const std::vector<uint8_t> big = radio::rawPayload(AX25_MAX_PACKET_LEN - 2U);

  m_mode   = 1U;
  m_duplex = true;
  m_tx     = false;

  hooks::g_dacOut.clear();

  ax25TX.setTXDelay(255U);
  REQUIRE_EQ(int(ax25TX.writeData(&big[0], uint16_t(big.size()))), 0);

  for (unsigned t = 0U; t < 4000U; t++) {
    ax25TX.process();
    io.interrupt();
  }

  CHECK_EQ(int(ax25TX.writeData(&big[0], uint16_t(big.size()))), 5);

  for (unsigned t = 0U; t < 24000U * 6U; t++) {
    ax25TX.process();
    io.interrupt();
  }

  const std::vector<uint16_t> wave = hooks::g_dacOut;
  hooks::g_dacOut.clear();
  m_tx     = false;
  m_duplex = (DUPLEX == 1);

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch), true, 1U), big));
}

TF_TEST(ax25_decodes_on_a_noisy_channel)
{
  /* Measured today: 100% to 200 LSB of peak noise. Assert everything at 100
     and at least 90% at 150, mirroring the mode 2 noise tests. */
  const std::vector<uint8_t> frame = radio::sampleUIFrame("NOISY CHANNEL TEST");

  unsigned at100 = 0U;
  unsigned at150 = 0U;
  const unsigned TRIALS = 20U;

  for (unsigned t = 0U; t < TRIALS; t++) {
    radio::Channel ch;
    ch.seed   = 5000U + t * 13U;
    ch.phase  = t % 5U;
    ch.invert = (t % 2U) != 0U;

    ch.noise = 100U;
    if (radio::decodedExactly(ax25Loopback(frame, ch), frame))
      at100++;

    ch.noise = 150U;
    if (radio::decodedExactly(ax25Loopback(frame, ch), frame))
      at150++;
  }

  CHECK_MSG(at100 == TRIALS, "100 LSB of noise cost " << (TRIALS - at100) << " of " << TRIALS);
  CHECK_MSG(at150 * 100U >= TRIALS * 90U, "150 LSB of noise decoded only " << at150 << " of " << TRIALS);
}

TF_TEST(ax25_tolerates_a_range_of_receive_levels)
{
  const std::vector<uint8_t> frame = radio::sampleUIFrame("LEVEL TEST");

  const double gains[] = { 0.25, 0.5, 2.0, 3.0 };

  for (unsigned i = 0U; i < sizeof(gains) / sizeof(gains[0]); i++) {
    radio::Channel ch;
    ch.gain = gains[i];

    CHECK_MSG(radio::decodedExactly(ax25Loopback(frame, ch), frame), "gain " << gains[i]);
  }
}

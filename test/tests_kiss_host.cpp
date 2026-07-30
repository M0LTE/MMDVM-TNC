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
 * The KISS host interface: CSerialPort::process() parsing what the host
 * sends, processMessage() dispatching it, and writeKISSData() escaping the
 * traffic going the other way. This is the one part of the firmware every
 * user's software talks to on every packet, so what goes in over the wire
 * here is fed byte for byte through the real parser, and what the firmware
 * queues is then carried over the virtual radio to prove it went out intact.
 */

#include "Config.h"
#include "Globals.h"
#include "KISSDefines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <algorithm>
#include <cstdlib>

namespace {

  /* A host-side KISS encoder, written independently of the firmware's, so the
     two implementations keep each other honest. */
  std::vector<uint8_t> kissEncode(uint8_t type, const std::vector<uint8_t>& content)
  {
    std::vector<uint8_t> out;

    out.push_back(KISS_FEND);
    out.push_back(uint8_t(type | (KISS_ADDRESS << 4)));

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

  void feed(const std::vector<uint8_t>& bytes)
  {
    hooks::g_kissRx.insert(hooks::g_kissRx.end(), bytes.begin(), bytes.end());
    serial.process();
  }

  int peakDeviation(const std::vector<uint16_t>& wave)
  {
    int peak = 0;
    for (size_t i = 0U; i < wave.size(); i++)
      peak = std::max(peak, std::abs(int(wave[i]) - int(radio::MID)));
    return peak;
  }

}

TF_TEST(kiss_data_frame_is_transmitted_and_decodes_back)
{
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  feed(kissEncode(KISS_TYPE_DATA, payload));

  const std::vector<uint16_t> wave = radio::runTX(2U);
  REQUIRE_MSG(wave.size() > 1000U, "the data frame never reached the transmitter");

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(kiss_reserved_bytes_survive_the_round_trip)
{
  /* A payload stuffed with the four bytes KISS has to escape. The host to
     modem direction exercises the parser's unescaping; the frame coming back
     from the receiver exercises writeKISSData()'s escaping. */
  std::vector<uint8_t> payload = radio::rawPayload(24U);
  const uint8_t reserved[8] = { KISS_FEND, KISS_FESC, KISS_TFEND, KISS_TFESC,
                                KISS_FESC, KISS_FEND, KISS_FEND, KISS_FESC };
  for (size_t i = 0U; i < 8U; i++)
    payload[i] = reserved[i];

  feed(kissEncode(KISS_TYPE_DATA, payload));

  const std::vector<uint16_t> wave = radio::runTX(2U);
  REQUIRE(wave.size() > 1000U);

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(kiss_output_is_escaped_on_the_wire)
{
  /* The loopback tests above cannot see a matched pair of escaping bugs, so
     pin the exact on-wire encoding down as well. */
  const uint8_t data[3] = { KISS_FEND, KISS_FESC, 0x41U };

  serial.writeKISSData(KISS_TYPE_DATA, data, 3U);

  const uint8_t expected[8] = { KISS_FEND, KISS_TYPE_DATA,
                                KISS_FESC, KISS_TFEND,
                                KISS_FESC, KISS_TFESC,
                                0x41U, KISS_FEND };

  REQUIRE_EQ(int(hooks::g_kissTx.size()), 8);
  for (size_t i = 0U; i < 8U; i++)
    CHECK_MSG(hooks::g_kissTx[i] == expected[i],
              "byte " << i << " is 0x" << std::hex << int(hooks::g_kissTx[i])
              << ", expected 0x" << int(expected[i]));
}

TF_TEST(kiss_frame_delivered_byte_by_byte_still_parses)
{
  /* Serial data arrives whenever it likes; the parser's state has to survive
     any split. */
  const std::vector<uint8_t> payload = radio::rawPayload(24U);
  const std::vector<uint8_t> wire    = kissEncode(KISS_TYPE_DATA, payload);

  for (size_t i = 0U; i < wire.size(); i++)
    feed(std::vector<uint8_t>(1U, wire[i]));

  const std::vector<uint16_t> wave = radio::runTX(2U);
  REQUIRE(wave.size() > 1000U);

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(kiss_wrong_address_is_ignored)
{
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  std::vector<uint8_t> wire = kissEncode(KISS_TYPE_DATA, payload);
  wire[1] = uint8_t(0x10U | KISS_TYPE_DATA);     /* KISS address 1, not ours */

  feed(wire);

  /* A second of air time: the DAC must stay at mid rail and PTT must stay
     released, because nothing should have been queued. */
  m_duplex = true;

  bool sawSignal = false;
  bool sawPTT    = false;

  for (unsigned t = 0U; t < 24000U; t++) {
    mode2TX.process();
    io.interrupt();

    if (hooks::g_dacOut.back() != radio::MID)
      sawSignal = true;
    if (hooks::g_ptt)
      sawPTT = true;
  }

  CHECK(!sawSignal);
  CHECK(!sawPTT);
}

TF_TEST(kiss_empty_frames_are_ignored)
{
  /* Back to back FENDs are the classic KISS keepalive/framing idiom; they
     must neither produce traffic nor upset the frame that follows. */
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  feed(std::vector<uint8_t>(3U, KISS_FEND));
  feed(kissEncode(KISS_TYPE_DATA, payload));
  feed(std::vector<uint8_t>(2U, KISS_FEND));

  const std::vector<uint16_t> wave = radio::runTX(2U);
  REQUIRE(wave.size() > 1000U);

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(kiss_tx_delay_command_stretches_the_preamble)
{
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  feed(kissEncode(KISS_TYPE_DATA, payload));
  const size_t before = radio::runTX(2U).size();
  REQUIRE(before > 1000U);

  feed(kissEncode(KISS_TYPE_TX_DELAY, std::vector<uint8_t>(1U, 80U)));
  CHECK(hooks::debugContains("Setting TX Delay to"));

  feed(kissEncode(KISS_TYPE_DATA, payload));
  const std::vector<uint16_t> wave = radio::runTX(2U);

  /* 50 more 10 ms units of preamble: 50 * 12 bytes * 20 samples per byte. */
  const long expected = 50L * 12L * 20L;
  const long grewBy   = long(wave.size()) - long(before);

  CHECK_MSG(std::labs(grewBy - expected) <= 200L,
            "preamble grew by " << grewBy << " samples, expected about " << expected);

  /* And the long preamble must not upset the receiver. */
  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

TF_TEST(kiss_command_with_the_wrong_length_is_ignored)
{
  feed(kissEncode(KISS_TYPE_TX_DELAY, std::vector<uint8_t>()));       /* no value */
  feed(kissEncode(KISS_TYPE_TX_DELAY, std::vector<uint8_t>(2U, 80U))); /* too many */

  CHECK(!hooks::debugContains("Setting TX Delay to"));
}

TF_TEST(kiss_unknown_frame_type_is_reported)
{
  feed(kissEncode(0x0BU, std::vector<uint8_t>()));

  CHECK(hooks::debugContains("Unhandled KISS frame type"));
}

TF_TEST(kiss_set_hardware_switches_mode_and_sample_rate)
{
  REQUIRE_EQ(int(m_mode), 2);
  REQUIRE_EQ(int(hooks::g_sampleRate), 24000);

  feed(kissEncode(KISS_TYPE_SET_HARDWARE, std::vector<uint8_t>(1U, 3U)));
  CHECK_EQ(int(m_mode), 3);
  CHECK_EQ(int(hooks::g_sampleRate), 48000);

  feed(kissEncode(KISS_TYPE_SET_HARDWARE, std::vector<uint8_t>(1U, 1U)));
  CHECK_EQ(int(m_mode), 1);
  CHECK_EQ(int(hooks::g_sampleRate), 24000);

  feed(kissEncode(KISS_TYPE_SET_HARDWARE, std::vector<uint8_t>(1U, 2U)));
  CHECK_EQ(int(m_mode), 2);
  CHECK_EQ(int(hooks::g_sampleRate), 24000);
}

TF_TEST(kiss_set_hardware_tx_level_scales_the_burst)
{
  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  feed(kissEncode(KISS_TYPE_DATA, payload));
  const int fullLevel = peakDeviation(radio::runTX(2U));
  REQUIRE(fullLevel > 200);

  /* The four byte SET_HARDWARE variant: RX level, mode 1 TX, mode 2 TX. */
  std::vector<uint8_t> levels;
  levels.push_back(128U);
  levels.push_back(128U);
  levels.push_back(64U);
  feed(kissEncode(KISS_TYPE_SET_HARDWARE, levels));

  /* Setting levels must not be mistaken for the one byte mode change. */
  CHECK_EQ(int(m_mode), 2);

  feed(kissEncode(KISS_TYPE_DATA, payload));
  const int halfLevel = peakDeviation(radio::runTX(2U));

  CHECK_MSG(std::abs(halfLevel * 2 - fullLevel) <= fullLevel / 8,
            "half level burst peaks at " << halfLevel << ", full level at " << fullLevel);
}

TF_TEST(kiss_data_with_ack_acks_only_after_transmission)
{
  /* The whole point of DATA_WITH_ACK is that the host learns when its frame
     has actually been transmitted, so in simplex mode the ack must not appear
     while the packet is still waiting for the channel or still going out. */
  REQUIRE(!m_duplex);

  const std::vector<uint8_t> payload = radio::rawPayload(32U);

  std::vector<uint8_t> content;
  content.push_back(0x12U);
  content.push_back(0x34U);
  content.insert(content.end(), payload.begin(), payload.end());

  feed(kissEncode(KISS_TYPE_DATA_WITH_ACK, content));

  bool rose        = false;
  bool fell        = false;
  bool ackTooEarly = false;

  for (unsigned t = 0U; t < 24000U * 20U && !fell; t++) {
    io.interrupt();
    io.process();
    mode2TX.process();

    if (hooks::g_ptt)
      rose = true;
    else if (rose)
      fell = true;

    /* Nothing may come back to the host before the burst has finished. */
    if (!fell && !hooks::g_kissTx.empty())
      ackTooEarly = true;
  }

  REQUIRE_MSG(rose, "the packet was never transmitted");
  REQUIRE_MSG(fell, "the transmission never finished");
  CHECK_MSG(!ackTooEarly, "the ack must wait for the end of the transmission");

  /* Let the transmitter notice it has finished and flush the token. */
  for (unsigned t = 0U; t < 16U; t++) {
    io.interrupt();
    io.process();
    mode2TX.process();
  }

  const std::vector<std::vector<uint8_t> > frames = hooks::kissFrames();
  REQUIRE_EQ(int(frames.size()), 1);
  REQUIRE_EQ(int(frames[0].size()), 3);
  CHECK_EQ(int(frames[0][0]), int(KISS_TYPE_ACK));

  /* The ack must carry back exactly the bytes the host sent. BPQ's ACKMODE
     puts the ackword on the wire low byte first in both directions (kiss.c:
     ENCBUFF[2] = ACKWORD & 0xff on send, RXMSG[1] | RXMSG[2] << 8 on
     receive) and silently drops an ack that matches no outstanding frame, so
     a byte swap here loses every ack. */
  CHECK_EQ(int(frames[0][1]), 0x12);
  CHECK_EQ(int(frames[0][2]), 0x34);
}

TF_TEST(kiss_oversized_frame_is_dropped_and_the_parser_recovers)
{
  /* The parser's buffer is 2000 bytes. A frame bigger than that -- line
     noise, a runaway host -- must be thrown away without touching memory past
     the buffer, and the next well formed frame must still get through. */
  std::vector<uint8_t> monster;
  monster.push_back(KISS_FEND);
  monster.push_back(KISS_TYPE_DATA);
  for (unsigned i = 0U; i < 2600U; i++)
    monster.push_back(0x55U);
  monster.push_back(KISS_FEND);

  feed(monster);

  const std::vector<uint8_t> payload = radio::rawPayload(32U);
  feed(kissEncode(KISS_TYPE_DATA, payload));

  const std::vector<uint16_t> wave = radio::runTX(2U);
  REQUIRE_MSG(wave.size() > 1000U, "the frame after the oversized one was lost");

  radio::Channel ch;
  CHECK(radio::decodedExactly(radio::demodulate(radio::applyChannel(wave, ch)), payload));
}

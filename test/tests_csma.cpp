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
 * Channel access. A simplex TNC handed a packet while another station is on
 * the air must sit on its hands until the channel clears -- that is CIO's
 * DCD / slot time / p-persistence machinery, which nothing else in the suite
 * exercises. The full station is pumped here: the other station's burst plays
 * into the ADC, and our own packet is handed to the transmitter in the middle
 * of it.
 *
 * CIO's p-persistence PRNG is seeded with fixed constants, so these tests are
 * deterministic even though the protocol is randomised on the air.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

namespace {

  struct StationRun {
    bool pttRose;
    bool pttFell;
    bool dcdSeen;
    bool pttWhileDCD;
    std::vector<uint16_t> transmitted;
  };

  /* Play `incoming` into the ADC and run the whole station until our own
     transmission has come and gone. `ours` goes to the transmitter either
     straight away, or -- the interesting case -- the moment the receiver
     declares the channel busy. */
  StationRun runStation(const std::vector<uint8_t>& ours,
                        const std::vector<uint16_t>& incoming,
                        bool queueImmediately)
  {
    StationRun r;
    r.pttRose     = false;
    r.pttFell     = false;
    r.dcdSeen     = false;
    r.pttWhileDCD = false;

    hooks::g_kissTx.clear();
    hooks::g_dacOut.clear();
    hooks::g_adcIn    = incoming;
    hooks::g_adcInPtr = 0U;

    m_mode = 2U;

    bool queued = queueImmediately;
    if (queueImmediately)
      REQUIRE_EQ(int(mode2TX.writeData(&ours[0], uint16_t(ours.size()))), 0);

    for (unsigned t = 0U; t < 24000U * 30U && !r.pttFell; t++) {
      io.interrupt();
      io.process();
      mode2TX.process();

      if (hooks::g_dcd)
        r.dcdSeen = true;

      if (!queued && hooks::g_dcd) {
        REQUIRE_EQ(int(mode2TX.writeData(&ours[0], uint16_t(ours.size()))), 0);
        queued = true;
      }

      if (hooks::g_ptt && hooks::g_dcd)
        r.pttWhileDCD = true;

      if (hooks::g_ptt)
        r.pttRose = true;
      else if (r.pttRose)
        r.pttFell = true;
    }

    r.transmitted = hooks::g_dacOut;

    return r;
  }

}

TF_TEST(csma_holds_ptt_while_the_channel_is_busy)
{
  REQUIRE(!m_duplex);

  const std::vector<uint8_t> theirs = radio::rawPayload(40U);
  const std::vector<uint8_t> ours   = radio::rawPayload(56U);

  /* modulate() leaves the station idle again, so it is safe to use it to
     manufacture the other station's burst first. */
  const std::vector<uint16_t> incoming = radio::modulate(&theirs[0], uint16_t(theirs.size()));
  REQUIRE(incoming.size() > 1000U);

  const StationRun r = runStation(ours, incoming, false);

  REQUIRE_MSG(r.dcdSeen, "the receiver never saw the other station at all");
  REQUIRE_MSG(r.pttRose, "our packet was never transmitted");
  REQUIRE_MSG(r.pttFell, "our transmission never finished");

  CHECK_MSG(!r.pttWhileDCD, "PTT was keyed while the channel was busy");

  /* Holding off must not have cost us the other station's frame... */
  CHECK(radio::decodedExactly(hooks::kissFrames(), theirs));

  /* ...nor our own, which went to air once the channel cleared. */
  CHECK(radio::decodedExactly(radio::demodulate(r.transmitted), ours));
}

TF_TEST(csma_full_duplex_transmits_without_waiting)
{
  const std::vector<uint8_t> theirs = radio::rawPayload(40U);
  const std::vector<uint8_t> ours   = radio::rawPayload(56U);

  const std::vector<uint16_t> incoming = radio::modulate(&theirs[0], uint16_t(theirs.size()));
  REQUIRE(incoming.size() > 1000U);

  m_duplex = true;              /* modulate() left it at the simplex default */

  const StationRun r = runStation(ours, incoming, true);

  REQUIRE_MSG(r.dcdSeen, "the receiver never saw the other station at all");
  REQUIRE_MSG(r.pttRose, "our packet was never transmitted");

  /* Full duplex means transmitting over the top of the incoming signal. */
  CHECK_MSG(r.pttWhileDCD, "full duplex should not have waited for the channel");

  /* Both frames still get through: theirs on our receiver... */
  CHECK(radio::decodedExactly(hooks::kissFrames(), theirs));

  /* ...and ours on the air. */
  CHECK(radio::decodedExactly(radio::demodulate(r.transmitted), ours));
}

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
 * The first packet after power on.
 *
 * CMode2RX::reset() puts the receiver into a known state, and it is called
 * after every packet, good or bad. It is not called before the first one --
 * the constructor is expected to leave the object ready.
 *
 * Every test here runs in its own forked process, so mode2RX really is
 * freshly constructed, and demodulate(..., false) leaves it that way rather
 * than tidying up first.
 */

#include "Config.h"
#include "Globals.h"

#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

TF_TEST(mode2_decodes_the_first_packet_after_power_on)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> burst  = radio::modulate(&payload[0], uint16_t(payload.size()));

  const std::vector<std::vector<uint8_t> > frames = radio::demodulate(burst, false);

  CHECK_MSG(radio::decodedExactly(frames, payload),
            "the first packet a freshly constructed receiver sees did not decode; "
            "got " << frames.size() << " frame(s)");
}

TF_TEST(mode2_decodes_the_first_packet_at_every_sampling_phase)
{
  /*
   * Whether the first packet survives should not depend on which of the five
   * sample phases it happens to arrive on.
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> clean  = radio::modulate(&payload[0], uint16_t(payload.size()));

  unsigned decoded = 0U;

  for (unsigned phase = 0U; phase < MODE2_RADIO_SYMBOL_LENGTH; phase++) {
    radio::Channel ch;
    ch.phase = phase;

    /* A forked process per test, so this is genuinely a cold receiver; the
       loop below deliberately does not reset it between phases either, which
       makes every iteration after the first a warm one. */
    const bool ok = radio::decodedExactly(
                      radio::demodulate(radio::applyChannel(clean, ch), phase != 0U), payload);
    if (ok)
      decoded++;

    CHECK_MSG(ok, "lost the packet at sampling phase " << phase
                  << (phase == 0U ? " (cold receiver)" : " (warm receiver)"));
  }

  CHECK_EQ(int(decoded), int(MODE2_RADIO_SYMBOL_LENGTH));
}

TF_TEST(mode2_first_packet_is_not_thrown_away_by_a_zero_threshold)
{
  /*
   * The specific failure: correlateSync() only measures the eye when
   * m_averagePtr holds NOAVEPTR, and processNone() then refuses the sync
   * unless m_thresholdVal reached 50. A receiver whose m_averagePtr did not
   * start at NOAVEPTR never takes the measurement, so the threshold stays at
   * zero and the sync is discarded.
   *
   * Observable on the debug port: the receiver reports a valid sync vector
   * and then does not go on to report a header at all.
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);
  const std::vector<uint16_t> burst  = radio::modulate(&payload[0], uint16_t(payload.size()));

  radio::demodulate(burst, false);

  const bool sawSync   = hooks::debugContains("valid sync vector");
  const bool sawHeader = hooks::debugContains("header is valid");

  CHECK_MSG(sawSync, "the receiver never found the sync vector at all");
  CHECK_MSG(sawHeader,
            "the receiver found the sync vector but never reached the header, "
            "which is what a threshold of zero looks like");
}

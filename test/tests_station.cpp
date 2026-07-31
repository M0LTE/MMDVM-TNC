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
 * The whole TNC through the real firmware entry points. Every other test
 * drives io/serial/the modems directly; this one calls the actual setup()
 * and loop() from MMDVM.cpp, so the top level wiring -- the order things are
 * polled in, the mode dispatch, the start-up path -- is executed too, and a
 * KISS frame makes the entire journey a real packet makes: host serial in,
 * over the air, back through the receiver, host serial out.
 */

#include "Config.h"
#include "Globals.h"
#include "KISSDefines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

/* MMDVM.cpp; its main() is renamed away but these are the real thing. */
void setup();
void loop();

TF_TEST(station_kiss_in_over_the_air_and_kiss_out_via_the_real_main_loop)
{
  setup();

  /* start() announces the firmware on the debug port. */
  CHECK(hooks::debugContains("MMDVM"));

  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  /* Hand the packet over as the host would. */
  std::vector<uint8_t> wire;
  wire.push_back(KISS_FEND);
  wire.push_back(KISS_TYPE_DATA);
  for (size_t i = 0U; i < payload.size(); i++) {
    const uint8_t c = payload[i];
    if (c == KISS_FEND) {
      wire.push_back(KISS_FESC);
      wire.push_back(KISS_TFEND);
    } else if (c == KISS_FESC) {
      wire.push_back(KISS_FESC);
      wire.push_back(KISS_TFESC);
    } else {
      wire.push_back(c);
    }
  }
  wire.push_back(KISS_FEND);

  hooks::g_kissRx.insert(hooks::g_kissRx.end(), wire.begin(), wire.end());

  m_duplex = true;
  hooks::g_dacOut.clear();

  /* Transmit: the real main loop against the sample interrupt. */
  bool rose = false;
  bool fell = false;

  for (unsigned t = 0U; t < 24000U * 10U && !fell; t++) {
    loop();
    io.interrupt();

    if (hooks::g_ptt)
      rose = true;
    else if (rose)
      fell = true;
  }

  REQUIRE_MSG(rose, "the packet never went to air");
  REQUIRE_MSG(fell, "the transmission never finished");

  /* Receive: play the burst straight back into the ADC. */
  const std::vector<uint16_t> burst = hooks::g_dacOut;
  REQUIRE(burst.size() > 1000U);

  hooks::g_kissTx.clear();
  hooks::g_adcIn    = burst;
  hooks::g_adcInPtr = 0U;

  for (size_t t = 0U; t < burst.size() + 8192U; t++) {
    loop();
    io.interrupt();
  }

  CHECK(radio::decodedExactly(hooks::kissFrames(), payload));
}

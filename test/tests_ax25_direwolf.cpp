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
 * Mode 1 against a different implementation. The fixture was generated with
 * Dire Wolf's gen_packets at 24 kHz:
 *
 *   gen_packets -r 24000 -n 20 -o direwolf-1200-AFSK-20-frames.wav
 *
 * which produces twenty numbered frames with steadily increasing noise --
 * Dire Wolf's own modem sensitivity benchmark. Until now the mode 1
 * receiver had only ever decoded this firmware's own transmitter, which
 * shares all of its assumptions; Dire Wolf's modulator shares none.
 *
 * Measured today: this receiver decodes 11 of the 20. Dire Wolf's atest
 * decodes 10 of the same file with its default profile, so the three
 * demodulator bank is genuinely competitive; the assertion is the floor.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "Radio.h"

#include <cstring>

TF_TEST(ax25_decodes_the_direwolf_noise_ramp)
{
  unsigned rate = 0U;
  const std::vector<int16_t> pcm =
    radio::loadWav("../testdata/direwolf-1200-AFSK-20-frames.wav", rate);

  REQUIRE_MSG(!pcm.empty(), "fixture missing; run from the test directory");
  REQUIRE_EQ(int(rate), 24000);

  const std::vector<std::vector<uint8_t> > frames =
    radio::demodulate(radio::toAdc(pcm, 800), true, 1U);

  CHECK_MSG(frames.size() >= 11U,
            "decoded " << frames.size() << " of 20; 11 is today's figure and the floor");

  /* The first, cleanest frame must be intact: Dire Wolf's built in message
     from WB2OSZ, carrying the fox. */
  REQUIRE(!frames.empty());
  const std::vector<uint8_t>& f = frames[0];

  const uint8_t source[7] = { 0xAEU, 0x84U, 0x64U, 0x9EU, 0xA6U, 0xB4U, 0xFFU };  /* WB2OSZ-15 */
  REQUIRE(f.size() > 40U);
  CHECK(::memcmp(&f[8], source, 7U) == 0);

  const char* fox = "The quick brown fox";
  bool found = false;
  for (size_t i = 1U; i + std::strlen(fox) <= f.size() && !found; i++)
    found = ::memcmp(&f[i], fox, std::strlen(fox)) == 0;
  CHECK_MSG(found, "the first frame's payload is damaged");
}

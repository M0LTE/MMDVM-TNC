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
 * The debug port.
 *
 * This is the only window into a running modem, and the receive path leans on
 * it heavily -- correlateSync() reports the correlation, position, both
 * symbol error counts and the byte error count on one line. If those fields
 * do not arrive as one line, or arrive truncated, the output actively
 * misleads whoever is reading it.
 */

#include "Config.h"
#include "Globals.h"

#include "framework.h"
#include "shim/TestHooks.h"

#include <string>

TF_TEST(debug_writes_one_line_per_message)
{
  serial.writeDebug("plain");

  CHECK_EQ(hooks::g_debugTx, std::string("plain\n"));
}

TF_TEST(debug_does_not_break_a_line_at_a_zero_value)
{
  serial.writeDebug("value", 0);

  CHECK_EQ(hooks::g_debugTx, std::string("value 0\n"));
}

TF_TEST(debug_keeps_every_field_of_a_message_together)
{
  /* Shaped like the sync report, which is the one that matters most:
     DEBUG6("Mode2RX: valid sync vector", corr, m_dataPtr, n1, n2, errs)
     where n1 or errs being zero is the normal, healthy case. */
  serial.writeDebug("sync", 17973, 3270, 0, 16, 0);

  CHECK_EQ(hooks::g_debugTx, std::string("sync 17973 3270 0 16 0\n"));
}

TF_TEST(debug_reports_negative_values)
{
  serial.writeDebug("levels", -190, 0, -1);

  CHECK_EQ(hooks::g_debugTx, std::string("levels -190 0 -1\n"));
}

TF_TEST(debug_reports_values_that_do_not_fit_in_16_bits)
{
  /* corr is a q31_t. A strong signal takes it past 32767, at which point a
     16 bit parameter reports a wrapped, negative, meaningless number. */
  serial.writeDebug("corr", 100000);

  CHECK_EQ(hooks::g_debugTx, std::string("corr 100000\n"));
}

TF_TEST(debug_reports_the_most_negative_value_without_overflow)
{
  serial.writeDebug("min", -2147483647 - 1);

  CHECK_EQ(hooks::g_debugTx, std::string("min -2147483648\n"));
}

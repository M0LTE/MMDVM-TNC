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
 * How much damage the sync vector tolerates.
 *
 * correlateSync() slices the sync vector into MODE2_SYNC_LENGTH_BYTES, counts
 * the bits that differ from MODE2_SYNC_BYTES, and accepts up to
 * MAX_SYNC_BIT_ERRS of them. That budget is the receiver's entire margin at
 * sync time, so all of it has to be available for real impairments.
 *
 * Errors are injected as magnitude flips: the low bit of a symbol's dibit,
 * which moves +3 <-> +1 or -1 <-> -3. The sign is untouched, so the 16 bit
 * sign correlator that gates correlateSync() still fires and only the byte
 * comparison sees the damage. baseline_sync_vector_symbol_mapping pins that
 * property down.
 */

#include "Config.h"
#include "Globals.h"

#include "Mode2Defines.h"

#include "framework.h"
#include "Radio.h"
#include "shim/TestHooks.h"

#include <cstring>

namespace {

  const unsigned PREAMBLE_BYTES = 36U;

  /*
   * Flip the low bit of the dibit carrying sync symbol `symbol`, turning a
   * +/-3 into a +/-1. Exactly one bit error against MODE2_SYNC_BYTES.
   */
  void injectSyncError(std::vector<uint8_t>& bytes, unsigned symbol)
  {
    const unsigned byteIndex = PREAMBLE_BYTES + (symbol / 4U);
    const unsigned shift     = 6U - 2U * (symbol % 4U);

    bytes[byteIndex] = uint8_t(bytes[byteIndex] ^ (1U << shift));
  }

  std::vector<uint16_t> burstWithSyncErrors(const std::vector<uint8_t>& payload,
                                            const unsigned* symbols, unsigned count)
  {
    std::vector<uint8_t> bytes = radio::onAirBytes(payload, PREAMBLE_BYTES);

    for (unsigned i = 0U; i < count; i++)
      injectSyncError(bytes, symbols[i]);

    return radio::modulateSymbols(radio::bytesToSymbols(bytes));
  }

  bool decodes(const std::vector<uint8_t>& payload, const unsigned* symbols, unsigned count)
  {
    return radio::decodedExactly(radio::demodulate(burstWithSyncErrors(payload, symbols, count)), payload);
  }

}

TF_TEST(mode2_sync_tolerates_a_single_bit_error)
{
  /*
   * MAX_SYNC_BIT_ERRS is 2, so one bit error in the sync vector must not cost
   * the packet. Sync is the receiver's first and only chance at a packet: a
   * sync vector rejected here is a packet that never existed.
   */
  const std::vector<uint8_t> payload  = radio::rawPayload(48U);
  const unsigned             errors[] = { 5U };

  CHECK_MSG(decodes(payload, errors, 1U),
            "one bit error in the sync vector lost the packet, "
            "against a documented budget of " << int(2));
}

TF_TEST(mode2_sync_tolerates_its_documented_bit_error_budget)
{
  const std::vector<uint8_t> payload  = radio::rawPayload(48U);
  const unsigned             errors[] = { 0U, 5U };

  CHECK_MSG(decodes(payload, errors, 2U),
            "two bit errors in the sync vector lost the packet, "
            "which is exactly the documented budget of " << int(2));
}

TF_TEST(mode2_sync_counts_an_error_in_the_final_symbol)
{
  /*
   * Two errors is at budget and must be accepted; adding a third in the LAST
   * symbol of the vector must push it over and be rejected. If the final
   * symbol is not being compared, both cases look identical to the receiver.
   */
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  const unsigned lastSymbol = MODE2_SYNC_LENGTH_SYMBOLS - 1U;

  const unsigned atBudget[]   = { 0U, 5U };
  const unsigned overBudget[] = { 0U, 5U, lastSymbol };

  const bool accepted = decodes(payload, atBudget, 2U);
  const bool rejected = !decodes(payload, overBudget, 3U);

  CHECK_MSG(accepted, "two bit errors were rejected");
  CHECK_MSG(rejected, "a third bit error, in symbol " << lastSymbol
                      << ", was not counted");
  CHECK_MSG(accepted && rejected,
            "the final symbol of the sync vector does not affect the outcome");
}

TF_TEST(mode2_sync_still_rejects_a_vector_that_is_over_budget)
{
  /*
   * A guard against fixing the tests by loosening the threshold: three bit
   * errors, none of them in the final symbol, must still be rejected.
   */
  const std::vector<uint8_t> payload  = radio::rawPayload(48U);
  const unsigned             errors[] = { 0U, 5U, 9U };

  CHECK_MSG(!decodes(payload, errors, 3U),
            "three bit errors were accepted against a budget of " << int(2));
}

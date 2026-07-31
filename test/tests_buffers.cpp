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
 * The plumbing everything else stands on: CRingBuffer carries every sample
 * and every byte between the interrupt handlers and the main loop, and
 * CTokenStore carries the DATA_WITH_ACK tokens. Neither had any direct
 * coverage.
 */

#include "Config.h"
#include "Globals.h"

#include "RingBuffer.h"
#include "TokenStore.h"
#include "Utils.h"

#include "framework.h"

TF_TEST(ringbuffer_is_fifo_across_a_wrap)
{
  CRingBuffer<uint8_t> rb(8U);
  uint8_t out = 0U;

  /* Fill most of it, half drain, refill: head and tail both wrap. */
  for (uint8_t i = 0U; i < 6U; i++)
    CHECK(rb.put(i));

  for (uint8_t i = 0U; i < 4U; i++) {
    CHECK(rb.get(out));
    CHECK_EQ(int(out), int(i));
  }

  for (uint8_t i = 6U; i < 12U; i++)
    CHECK(rb.put(i));

  for (uint8_t i = 4U; i < 12U; i++) {
    CHECK(rb.get(out));
    CHECK_EQ(int(out), int(i));
  }

  CHECK(!rb.get(out));
}

TF_TEST(ringbuffer_accounts_for_space_and_data)
{
  CRingBuffer<uint8_t> rb(8U);
  uint8_t out = 0U;

  CHECK_EQ(int(rb.getData()), 0);
  CHECK_EQ(int(rb.getSpace()), 8);

  /* The two views must stay complementary through fill, drain and wrap. */
  for (uint8_t i = 0U; i < 8U; i++) {
    rb.put(i);
    CHECK_EQ(int(rb.getData()), int(i + 1U));
    CHECK_EQ(int(rb.getData() + rb.getSpace()), 8);
  }

  CHECK_EQ(int(rb.getSpace()), 0);

  for (uint8_t i = 0U; i < 8U; i++) {
    rb.get(out);
    CHECK_EQ(int(rb.getSpace()), int(i + 1U));
    CHECK_EQ(int(rb.getData() + rb.getSpace()), 8);
  }
}

TF_TEST(ringbuffer_put_on_full_fails_and_latches_overflow)
{
  CRingBuffer<uint8_t> rb(8U);
  uint8_t out = 0U;

  for (uint8_t i = 0U; i < 8U; i++)
    CHECK(rb.put(i));

  CHECK(!rb.put(0x99U));
  CHECK(rb.hasOverflowed());
  CHECK(!rb.hasOverflowed());          /* reading it clears it */

  /* The refused item must not have damaged the contents. */
  CHECK(rb.get(out));
  CHECK_EQ(int(out), 0);

  CHECK(rb.put(0x42U));

  for (uint8_t i = 1U; i < 8U; i++) {
    CHECK(rb.get(out));
    CHECK_EQ(int(out), int(i));
  }

  CHECK(rb.get(out));
  CHECK_EQ(int(out), 0x42);

  rb.put(0x77U);
  rb.reset();
  CHECK_EQ(int(rb.getData()), 0);
  CHECK(!rb.get(out));
}

TF_TEST(ringbuffer_peek_shows_the_next_item_without_taking_it)
{
  CRingBuffer<uint8_t> rb(8U);
  uint8_t out = 0U;

  rb.put(0x11U);
  rb.put(0x22U);

  CHECK_EQ(int(rb.peek()), 0x11);
  CHECK_EQ(int(rb.getData()), 2);      /* peek consumed nothing */

  rb.get(out);
  CHECK_EQ(int(rb.peek()), 0x22);
}

TF_TEST(countbits_matches_known_answers)
{
  /* countBits16 sits inside the sync correlator's error budget arithmetic;
     an off by one here is a receiver that is too strict or too loose. */
  CHECK_EQ(int(countBits8(0x00U)), 0);
  CHECK_EQ(int(countBits8(0xFFU)), 8);
  CHECK_EQ(int(countBits8(0xA5U)), 4);

  CHECK_EQ(int(countBits16(0x0000U)), 0);
  CHECK_EQ(int(countBits16(0xFFFFU)), 16);
  CHECK_EQ(int(countBits16(0xA5A5U)), 8);
  CHECK_EQ(int(countBits16(0x8001U)), 2);

  CHECK_EQ(int(countBits32(0x00000000U)), 0);
  CHECK_EQ(int(countBits32(0xFFFFFFFFU)), 32);
  CHECK_EQ(int(countBits32(0xA5A5A5A5U)), 16);

  CHECK_EQ(int(countBits64(0ULL)), 0);
  CHECK_EQ(int(countBits64(~0ULL)), 64);
  CHECK_EQ(int(countBits64(0xA5A5A5A5A5A5A5A5ULL)), 32);
}

TF_TEST(tokenstore_capacity_iteration_reset_and_clear)
{
  CTokenStore store;
  uint16_t token = 0U;

  for (uint16_t i = 0U; i < 20U; i++)
    CHECK(store.add(uint16_t(1000U + i)));

  CHECK(!store.add(9999U));            /* twenty is the limit */

  /* Iteration returns everything in the order it went in. */
  store.reset();
  for (uint16_t i = 0U; i < 20U; i++) {
    CHECK(store.next(token));
    CHECK_EQ(int(token), int(1000U + i));
  }
  CHECK(!store.next(token));

  /* reset() rewinds the cursor without dropping anything. */
  store.reset();
  CHECK(store.next(token));
  CHECK_EQ(int(token), 1000);

  /* clear() empties it and makes room again. */
  store.clear();
  CHECK(!store.next(token));
  CHECK(store.add(7U));
  store.reset();
  CHECK(store.next(token));
  CHECK_EQ(int(token), 7);
}

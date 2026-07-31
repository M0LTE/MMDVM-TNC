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
 * Reflashing without touching the board.
 *
 * The STM32's factory ROM bootloader is normally entered by holding BOOT0
 * high through a reset, which on some boards -- the BI7JTA V3F4 among them
 * -- cannot be driven reliably from the host's GPIO lines, leaving physical
 * unplugging as the only way in. But the ROM can also simply be jumped to.
 * The host sends a magic KISS frame; requestBootloader() notes it in a RAM
 * word the startup code is told not to touch and takes a system reset; and
 * checkBootloader(), called first thing on the way back up, sees the note
 * and jumps into the ROM, which then listens on the same USART the host is
 * already connected to. stm32flash needs no BOOT0 or reset lines at all.
 *
 * BOOT0-through-a-power-cycle remains the recovery path for a board whose
 * firmware is too broken to take the request.
 */

#include "Config.h"
#include "Globals.h"

#if defined(STM32F4XX) || defined(STM32F7XX)

#if defined(STM32F4XX)
#include "stm32f4xx.h"
const uint32_t SYSTEM_MEMORY = 0x1FFF0000U;
#else
#include "stm32f7xx.h"
const uint32_t SYSTEM_MEMORY = 0x1FF00000U;
#endif

const uint32_t BOOTLOADER_MAGIC = 0x544F4F42U;   // "BOOT"

// .noinit is excluded from the startup zeroing of .bss, so this survives
// NVIC_SystemReset() and carries the request across it.
__attribute__((section(".noinit"))) static uint32_t m_bootloaderMagic;

void requestBootloader()
{
  m_bootloaderMagic = BOOTLOADER_MAGIC;

  NVIC_SystemReset();
}

void checkBootloader()
{
  if (m_bootloaderMagic != BOOTLOADER_MAGIC)
    return;

  m_bootloaderMagic = 0U;

  // Wind the clocks back to their reset state: SystemInit has already moved
  // the system clock onto the PLL, and the ROM expects to start from HSI.
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    ;

  RCC->CFGR = 0U;
  while ((RCC->CFGR & RCC_CFGR_SWS) != 0U)
    ;

  RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL  = 0U;

  SCB->VTOR = SYSTEM_MEMORY;

  __set_MSP(*(volatile uint32_t*)SYSTEM_MEMORY);

  void (*bootloader)() = (void (*)())(*(volatile uint32_t*)(SYSTEM_MEMORY + 4U));
  bootloader();
}

#endif

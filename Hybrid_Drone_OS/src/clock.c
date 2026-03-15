/**
 * @file    clock.c
 * @brief   Implementation of system clock, SysTick, FPU, and delay — STM32F411.
 *
 * Implementation notes
 *  - system_ticks and SystemCoreClock are defined here (single definition);
 *    all other translation units access them through the 'extern' declarations
 *    in clock.h.
 *  - SysTick is NOT configured inside SystemClock_Config().  The caller
 *    (typically main()) must invoke SysTick_Config(SystemCoreClock / 1000)
 *    AFTER SystemClock_Config() returns so that the 1 ms reload value is
 *    calculated from the updated 100 MHz SystemCoreClock.
 *  - PLL arithmetic:  SYSCLK = HSE × (N / M) / P
 *                             = 25 MHz × (200 / 25) / 2 = 100 MHz.
 */

#include "clock.h"

/* =========================================================================
 *  MODULE-LEVEL VARIABLE DEFINITIONS
 *
 *  Declared 'extern' in clock.h; defined exactly once here.
 * ========================================================================= */

/** Millisecond counter — written by SysTick_Handler(), read by delay_ms(). */
volatile uint32_t system_ticks = 0;

/**
 * Core clock frequency in Hz.
 * Initialised to the HSI default (16 MHz); updated to 100 MHz by
 * SystemClock_Config() after the PLL locks and is switched to SYSCLK.
 */
uint32_t SystemCoreClock = 16000000;

/* =========================================================================
 *  FUNCTION IMPLEMENTATIONS
 * ========================================================================= */

/* -------------------------------------------------------------------------
 *  SystemClock_Config
 * ------------------------------------------------------------------------- */
void SystemClock_Config(void) {

    /* --- 1. Start HSE and wait for ready --------------------------------- */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));     /* Stall until HSERDY = 1.       */

    /* --- 2. Flash access control for 100 MHz ----------------------------- */
    /*
     * At Vdd > 2.7 V and HCLK > 90 MHz the STM32F411 reference manual
     * (RM0383, Table 6) mandates 3 wait-states.  Enabling the prefetch
     * buffer and both caches avoids the performance penalty of the extra
     * wait-states on the ART Accelerator.
     */
    FLASH->ACR = FLASH_ACR_PRFTEN     /* Prefetch buffer enable.             */
               | FLASH_ACR_ICEN       /* Instruction cache enable.           */
               | FLASH_ACR_DCEN       /* Data cache enable.                  */
               | FLASH_ACR_LATENCY_3WS; /* 3 wait-states (90 < HCLK ≤ 100). */

    /* --- 3. APB1 prescaler = /2 (PCLK1 = 50 MHz, max for APB1) --------- */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* --- 4. Configure PLL: HSE source, M=25, N=200, P=2 → 100 MHz ------- */
    /*
     * Clear previous PLL source and M/N multiplier fields before writing.
     *  Bit 22      : PLLSRC (0 = HSI, 1 = HSE).
     *  Bits [5:0]  : PLLM — input divider (divide HSE from 25 MHz → 1 MHz VCO input).
     *  Bits [14:6] : PLLN — VCO multiplier (1 MHz × 200 = 200 MHz VCO output).
     *  Bits [17:16]: PLLP — output divider (200 MHz / 2 = 100 MHz SYSCLK).
     *                       Value 0b00 in PLLP field means ÷2 (RM0383 §6.3.2).
     */
    RCC->PLLCFGR &= ~((1U << 22) | (0x7FFFU << 0));    /* Clear SRC + M + N. */
    RCC->PLLCFGR  =  (25U  <<  0)  /* PLLM = 25: VCO input = 25/25 = 1 MHz. */
                  |  (200U <<  6)  /* PLLN = 200: VCO output = 1 × 200 MHz.  */
                  |  (1U   << 22); /* PLLSRC = 1: select HSE.                */
    RCC->PLLCFGR &= ~(3U << 16);   /* PLLP = 00: output divider = ÷2 (100 MHz). */

    /* --- 5. Enable PLL and wait for lock --------------------------------- */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));     /* Stall until PLLRDY = 1.       */

    /* --- 6. Switch SYSCLK source to PLL ---------------------------------- */
    /*
     * SW[1:0] = 10 selects the PLL output as SYSCLK (RM0383 §6.3.3).
     * SWS[1:0] (bits [3:2]) mirrors the active clock source; poll until
     * the hardware confirms the switch is complete.
     */
    RCC->CFGR |= (2U << 0);                /* SW = 0b10: select PLL.         */
    while ((RCC->CFGR & (3U << 2)) != (2U << 2)); /* Wait for SWS = 0b10.   */

    /* --- 7. Update software copy of core clock --------------------------- */
    SystemCoreClock = 100000000U;
}

/* -------------------------------------------------------------------------
 *  SysTick_Handler
 * ------------------------------------------------------------------------- */
void SysTick_Handler(void) {
    /*
     * The SysTick peripheral fires this ISR every reload period (1 ms when
     * configured by SysTick_Config(SystemCoreClock / 1000) in main()).
     * Declared volatile in clock.h to prevent the compiler from caching it
     * in a register across the ISR boundary.
     */
    system_ticks++;
}

/* -------------------------------------------------------------------------
 *  FPU_Enable
 * ------------------------------------------------------------------------- */
void FPU_Enable(void) {
    /*
     * On Cortex-M4F, bits [23:20] of SCB->CPACR control access to CP10
     * and CP11 (the single-precision and double-precision FPU register banks).
     * Writing 0b11 to each pair grants full access from both privileged and
     * unprivileged thread mode.  Without this, any FPU instruction triggers
     * a UsageFault (NOCP — No Coprocessor).
     *
     *  Bit positions:
     *   [21:20] = CP10 access level.
     *   [23:22] = CP11 access level.
     *   0xF = 0b1111 → full access to both.
     */
    SCB->CPACR |= (0xFU << 20);
}

/* -------------------------------------------------------------------------
 *  delay_ms
 * ------------------------------------------------------------------------- */
void delay_ms(uint32_t ms) {
    /*
     * Capture the current tick count before entering the busy-wait loop.
     * The subtraction (system_ticks - start_tick) uses unsigned arithmetic,
     * so the comparison remains correct even when system_ticks wraps around
     * from UINT32_MAX to 0 — the difference will still be the elapsed ticks.
     */
    uint32_t start_tick = system_ticks;
    while ((system_ticks - start_tick) < ms);
}

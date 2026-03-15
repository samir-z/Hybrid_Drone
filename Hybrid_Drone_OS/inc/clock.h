/**
 * @file    clock.h
 * @brief   System clock, SysTick, and FPU configuration — bare-metal STM32F411 (CMSIS / C11).
 *
 * Design constraints
 *  - Pure C11 procedural; no C++ constructs, no HAL, no Arduino.
 *  - Only external dependencies: <stdint.h>, "stm32f4xx.h".
 *  - System clock configured to 100 MHz using the internal PLL sourced from HSE (25 MHz crystal).
 *  - SysTick configured for 1 ms interrupts, incrementing a global millisecond counter.
 *  - FPU enabled for use in the MPU6050 scaling function and other floating-point operations.
 *
 * Default configuration applied by SystemClock_Config()
 *  - Clock source  : HSE (25 MHz external crystal) fed into the PLL.
 *  - PLL parameters: M = 25, N = 200, P = 2  → VCO = 200 MHz, SYSCLK = 100 MHz.
 *  - AHB  prescaler: 1   (HCLK  = SYSCLK = 100 MHz).
 *  - APB1 prescaler: 2   (PCLK1 = 50 MHz  — max allowed for STM32F411 APB1).
 *  - APB2 prescaler: 1   (PCLK2 = 100 MHz — max allowed for STM32F411 APB2).
 *  - Flash latency : 3 wait-states (required for Vdd > 2.7 V and HCLK > 90 MHz).
 *  - SysTick source: HCLK (100 MHz), reload value computed for 1 ms period.
 *  - FPU           : CPACR bits [23:20] set to 0xF — full access to CP10 and CP11.
 *
 * @note  CHANGE REQUIRED — Author field: replace the placeholder below with your
 *        full name and/or alias before committing to a shared repository.
 *
 * @author  Joshua (Samir Zambrano)
 * @standard JPL C Coding Standard / MISRA-C:2012 (advisory deviations noted)
 */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>
#include "stm32f4xx.h"

/* =========================================================================
 *  GLOBAL STATE
 *
 *  Definitions live in clock.c.  All other translation units that include
 *  this header receive only the 'extern' declaration, preventing multiple-
 *  definition linker errors.
 * ========================================================================= */

/**
 * Millisecond tick counter incremented by SysTick_Handler() every 1 ms.
 * Declared volatile because it is written in an ISR and read in task context.
 */
extern volatile uint32_t system_ticks;

/**
 * Core clock frequency in Hz.  Initialised to 16 000 000 (HSI default) and
 * updated to 100 000 000 by SystemClock_Config() after PLL lock.
 */
extern uint32_t SystemCoreClock;

/* =========================================================================
 *  PUBLIC API
 * ========================================================================= */

/**
 * @brief  Configure the system clock to 100 MHz via the PLL.
 *
 *  Sequence:
 *   1. Enable HSE oscillator and wait for HSERDY.
 *   2. Set Flash ACR: prefetch, instruction cache, data cache, 3 wait-states.
 *   3. Set APB1 prescaler to /2 (PCLK1 = 50 MHz).
 *   4. Configure PLL: source = HSE, M = 25, N = 200, P = 2 → SYSCLK = 100 MHz.
 *   5. Enable PLL and wait for PLLRDY.
 *   6. Switch SYSCLK to PLL output and wait for SWS confirmation.
 *   7. Update SystemCoreClock = 100 000 000.
 *
 * @note  Must be called before any peripheral that depends on SYSCLK/HCLK,
 *        including SysTick and UART baud-rate calculation.
 */
void SystemClock_Config(void);

/**
 * @brief  SysTick interrupt service routine — 1 ms periodic tick.
 *
 *  Increments the global @p system_ticks counter on every interrupt.
 *  The SysTick reload value must be configured externally (e.g., by
 *  SysTick_Config() or direct SYST_RVR write) before this handler fires.
 *
 * @note  This symbol overrides the weak default handler defined by CMSIS.
 *        Do NOT call it directly from application code.
 */
void SysTick_Handler(void);

/**
 * @brief  Enable the Cortex-M4 Floating-Point Unit (FPU).
 *
 *  Sets bits [23:20] of SCB->CPACR to 0b1111, granting full privileged and
 *  unprivileged access to coprocessors CP10 (single-precision FPU) and CP11
 *  (double-precision element of the FPU on Cortex-M4F).
 *
 * @note  Must be called before any floating-point instruction is executed;
 *        recommended as the very first call in main(), before SystemClock_Config().
 */
void FPU_Enable(void);

/**
 * @brief  Blocking delay using the SysTick millisecond counter.
 *
 *  Captures @p system_ticks at entry and busy-waits until the counter has
 *  advanced by at least @p ms.  The delay is accurate to ±1 ms due to the
 *  non-atomic read of the volatile counter.
 *
 * @param[in]  ms  Number of milliseconds to stall (0 returns immediately).
 *
 * @warning  This is a busy-wait; it blocks the CPU for the full duration.
 *           Do not call from an ISR context.
 * @warning  Maximum safe delay is UINT32_MAX − current_tick ms.  Counter
 *           wrap-around is handled correctly by unsigned subtraction.
 */
void delay_ms(uint32_t ms);

#endif /* CLOCK_H */

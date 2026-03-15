/**
 * @file    pwm.h
 * @brief   TIM2 four-channel PWM driver — bare-metal STM32F411 (CMSIS / C11).
 *
 * Design constraints
 *  - Pure C11 procedural; no C++ constructs, no HAL, no Arduino.
 *  - Only external dependencies: <stdint.h>, "stm32f4xx.h".
 *  - Operates exclusively on TIM2 channels 1–4, mapped to PA0–PA3 (AF1).
 *  - PWM frequency and resolution are compile-time fixed by PSC and ARR.
 *  - All four channels share a single timer base; frequency is global.
 *
 * Default configuration applied by PWM_Init()
 *  - GPIO pins    : PA0 (CH1), PA1 (CH2), PA2 (CH3), PA3 (CH4) — push-pull, AF1.
 *  - Timer clock  : PCLK1 × 2 = 100 MHz (APB1 timer clock with /2 prescaler).
 *  - Prescaler    : PSC = 0  → timer ticks at 100 MHz.
 *  - Auto-reload  : ARR = 4999 → PWM period = 5000 ticks = 50 µs → f = 20 kHz.
 *  - Duty cycle   : CCRx range [0, 5000] maps linearly to [0 %, 100 %].
 *  - PWM mode     : Mode 1 (output HIGH while CNT < CCRx) with preload enabled.
 *
 * Typical use — brushless ESC control
 *  - ESCs using a digital (20 kHz) PWM protocol accept duty-cycle values
 *    directly.  Map the desired throttle [0 %, 100 %] to CCRx [0, 5000].
 *
 * @note  CHANGE REQUIRED — Author field: replace the placeholder below with
 *        your full name and/or alias before committing to a shared repository.
 *
 * @author  Joshua (Samir Zambrano)
 * @standard JPL C Coding Standard / MISRA-C:2012 (advisory deviations noted)
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "stm32f4xx.h"

/* =========================================================================
 *  CONFIGURATION CONSTANTS
 * ========================================================================= */

/** TIM2 auto-reload value; defines the PWM period (100 MHz / 5000 = 20 kHz). */
#define PWM_ARR_VALUE   (5000U)

/* =========================================================================
 *  PUBLIC API
 * ========================================================================= */

/**
 * @brief  Initialize TIM2 for four-channel 20 kHz PWM output.
 *
 *  Sequence:
 *   1. Enable GPIOA clock (AHB1).
 *   2. Configure PA0–PA3 as push-pull alternate function, AF1 (TIM2).
 *   3. Enable TIM2 clock (APB1).
 *   4. Set PSC = 0 (timer clock = APB1 timer clock = 100 MHz).
 *   5. Set ARR = PWM_ARR_VALUE − 1 (period = 5000 ticks = 50 µs).
 *   6. Configure CCMR1/CCMR2: PWM Mode 1, output compare preload enabled.
 *   7. Enable all four capture/compare outputs via CCER.
 *   8. Start the counter (CR1_CEN).
 *
 * @note  All channels are initialised with CCRx = 0 (0 % duty cycle).
 *        Call PWM_SetDutyCycle() to set the desired output level.
 * @note  SystemClock_Config() must be called first so that PCLK1 = 50 MHz
 *        (APB1 timer clock = 100 MHz) when TIM2 is configured.
 */
void PWM_Init(void);

/**
 * @brief  Set the duty cycle of a TIM2 PWM channel.
 *
 *  Writes the given value directly to the corresponding capture/compare
 *  register (CCRx).  The duty cycle percentage is:
 *    duty [%] = (duty_cycle / PWM_ARR_VALUE) × 100
 *
 * @param[in]  channel     TIM2 channel number: 1, 2, 3, or 4.
 *                         Values outside [1, 4] are silently ignored.
 * @param[in]  duty_cycle  Raw CCR value in range [0, PWM_ARR_VALUE].
 *                         Values exceeding PWM_ARR_VALUE are clamped to
 *                         PWM_ARR_VALUE (100 % duty cycle).
 *
 * @note  The update takes effect at the start of the next PWM period
 *        because output compare preload is enabled.
 */
void PWM_SetDutyCycle(uint8_t channel, uint16_t duty_cycle);

#endif /* PWM_H */

/**
 * @file    pwm.c
 * @brief   Implementation of the TIM2 four-channel PWM driver — STM32F411.
 *
 * Implementation notes
 *  - Timer clock derivation:
 *      APB1 prescaler = /2  → PCLK1 = 50 MHz.
 *      When APB1 prescaler ≠ 1, the STM32F4 timer clock = PCLK1 × 2 = 100 MHz
 *      (RM0383 §6.2, "Timer clock").
 *  - PSC = 0 → no prescaling → TIM2 counter increments at 100 MHz.
 *  - ARR = 4999 → period = 5000 ticks = 5000 / 100 MHz = 50 µs → f = 20 kHz.
 *  - CCMR1/CCMR2 bit layout (per channel pair):
 *      OCxM  [6:4] / [14:12] = 0b110 (6) → PWM Mode 1
 *                               (output HIGH while CNT < CCRx).
 *      OCxPE [3]   / [11]    = 1         → preload enable
 *                               (CCRx shadow register; update on UEV).
 *  - PWM_SetDutyCycle() clamps duty_cycle to PWM_ARR_VALUE (constant in
 *    pwm.h) rather than to the raw TIM2->ARR, keeping the guard independent
 *    of runtime register changes.
 */

#include "pwm.h"

/* =========================================================================
 *  FUNCTION IMPLEMENTATIONS
 * ========================================================================= */

/* -------------------------------------------------------------------------
 *  PWM_Init
 * ------------------------------------------------------------------------- */
void PWM_Init(void) {

    /* --- 1. Enable GPIOA clock and configure PA0–PA3 -------------------- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /*
     * Clear and set MODER for PA0–PA3 to alternate function (MODER = 0b10).
     * Each pin occupies 2 bits; PA0→bits[1:0], PA1→bits[3:2], PA2→bits[5:4],
     * PA3→bits[7:6].
     */
    GPIOA->MODER &= ~((3U << 0) | (3U << 2) | (3U << 4) | (3U << 6));
    GPIOA->MODER |=  (2U << 0) | (2U << 2) | (2U << 4) | (2U << 6);

    /*
     * AFR[0] controls pins 0–7 with 4-bit fields.
     * AF1 is the TIM1/TIM2 alternate function for PA0–PA3 (RM0383 Table 9).
     * PA0→bits[3:0], PA1→bits[7:4], PA2→bits[11:8], PA3→bits[15:12].
     */
    GPIOA->AFR[0] &= ~((0xFU << 0) | (0xFU << 4) | (0xFU << 8) | (0xFU << 12));
    GPIOA->AFR[0] |=  (1U << 0) | (1U << 4) | (1U << 8) | (1U << 12);

    /* --- 2. Enable TIM2 clock and configure the timer base --------------- */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* PSC = 0: counter clock = APB1 timer clock = 100 MHz (no prescaling). */
    TIM2->PSC = 1U - 1U;

    /*
     * ARR = 4999: the counter resets after 5000 ticks.
     * PWM frequency = 100 MHz / 5000 = 20 kHz.
     * Duty cycle range: CCRx ∈ [0, 5000] maps linearly to [0 %, 100 %].
     */
    TIM2->ARR = 5000U - 1U;

    /* --- 3. Configure PWM Mode 1 with preload on all four channels -------- */
    /*
     * CCMR1 controls CH1 (lower byte) and CH2 (upper byte):
     *   Bits[6:4]   = OCM1  = 6 (0b110): PWM Mode 1 for CH1.
     *   Bit[3]      = OC1PE = 1         : preload enable for CH1.
     *   Bits[14:12] = OCM2  = 6 (0b110): PWM Mode 1 for CH2.
     *   Bit[11]     = OC2PE = 1         : preload enable for CH2.
     */
    TIM2->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);

    /* CCMR2 mirrors CCMR1 encoding for CH3 and CH4. */
    TIM2->CCMR2 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);

    /* --- 4. Enable capture/compare outputs for all four channels --------- */
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* --- 5. Start the counter -------------------------------------------- */
    /*
     * Setting CEN starts the free-running up-counter.  All CCRx registers
     * default to 0, so all channels output 0 % duty cycle until
     * PWM_SetDutyCycle() is called.
     */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* -------------------------------------------------------------------------
 *  PWM_SetDutyCycle
 * ------------------------------------------------------------------------- */
void PWM_SetDutyCycle(uint8_t channel, uint16_t duty_cycle) {

    /* Clamp to maximum (100 % duty cycle = ARR + 1 = 5000 ticks). */
    if (duty_cycle > PWM_ARR_VALUE) {
        duty_cycle = PWM_ARR_VALUE;
    }

    /*
     * Write the clamped value to the shadow CCR register.
     * Because output compare preload is enabled (OCxPE = 1 in PWM_Init),
     * the new value takes effect at the next update event (counter overflow),
     * which prevents glitches on the output waveform mid-cycle.
     */
    switch (channel) {
        case 1U: TIM2->CCR1 = duty_cycle; break;
        case 2U: TIM2->CCR2 = duty_cycle; break;
        case 3U: TIM2->CCR3 = duty_cycle; break;
        case 4U: TIM2->CCR4 = duty_cycle; break;
        default: break;     /* Invalid channel — silently ignored.           */
    }
}

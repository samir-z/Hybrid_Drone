/**
 * @file    uart.c
 * @brief   Implementation of the USART1 transmit driver — STM32F411.
 *
 * Implementation notes
 *  - Baud rate accuracy at 100 MHz SYSCLK:
 *      BRR = SystemCoreClock / 115200 = 100 000 000 / 115200 ≈ 868.
 *      Actual baud rate = 100 000 000 / 868 ≈ 115 207 bps → +0.006 % error.
 *      This is well within the ±2 % UART tolerance.
 *  - UART1_SendInt() builds the decimal string in reverse order (least-
 *    significant digit first) into a local stack buffer, then transmits
 *    the buffer in reverse to produce the correct left-to-right output.
 *    No dynamic allocation or libc dependency.
 *  - UART1_SendFloat() extracts each decimal digit by multiplying the
 *    remaining fractional part by 10 and truncating.  Accuracy degrades
 *    for decimal_places > 7 due to single-precision FPU representation
 *    limits (~7 significant decimal digits for float).
 *  - The TXE (Transmit-data-register Empty) flag indicates that the
 *    contents of USART1->DR have been copied to the transmit shift
 *    register, so a new byte can be written without overwriting the
 *    previous one.  TC (Transfer Complete) is not polled here — TXE is
 *    sufficient for back-to-back character transmission.
 */

#include "uart.h"
#include <stdbool.h>

/* =========================================================================
 *  FUNCTION IMPLEMENTATIONS
 * ========================================================================= */

/* -------------------------------------------------------------------------
 *  UART1_Init
 * ------------------------------------------------------------------------- */
void UART1_Init(void) {

    /* --- 1. Enable peripheral clocks ------------------------------------- */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  /* USART1 on APB2.               */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   /* GPIOA on AHB1.                */

    /* --- 2. Configure PA9 (TX) and PA10 (RX) as AF7 push-pull ----------- */
    /*
     * MODER: 0b10 = alternate function for PA9 (bits[19:18]) and PA10 (bits[21:20]).
     */
    GPIOA->MODER &= ~((3U << 18) | (3U << 20));
    GPIOA->MODER |=  (2U << 18) | (2U << 20);

    /*
     * AFR[1] controls pins 8–15 with 4-bit fields.
     * AF7 is the USART1/2/3 alternate function (RM0383 Table 9).
     * PA9  → bits[7:4]   of AFR[1].
     * PA10 → bits[11:8]  of AFR[1].
     */
    GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
    GPIOA->AFR[1] |=  (7U << 4) | (7U << 8);

    /* --- 3. Set baud rate ------------------------------------------------ */
    /*
     * USART1 is clocked from PCLK2 = SYSCLK = 100 MHz (APB2 prescaler = /1).
     * BRR uses the integer divider mode: BRR = f_PCLK / baud_rate.
     * Integer division truncates; at 100 MHz the error is negligible (<0.01 %).
     */
    USART1->BRR = SystemCoreClock / 115200U;

    /* --- 4. Enable transmitter, receiver, and USART peripheral ---------- */
    /*
     * TE (Transmitter Enable): activates the TX pin and sends an idle frame.
     * RE (Receiver Enable)   : activates the RX sampling logic.
     * UE (USART Enable)      : enables the peripheral; must be set last.
     */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* -------------------------------------------------------------------------
 *  UART1_SendChar
 * ------------------------------------------------------------------------- */
void UART1_SendChar(char c) {
    /*
     * Wait until TXE = 1 (DR has been moved to the shift register and is
     * ready to accept the next byte), then write the character to DR.
     */
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
}

/* -------------------------------------------------------------------------
 *  UART1_SendString
 * ------------------------------------------------------------------------- */
void UART1_SendString(const char *str) {
    /* Iterate until the null terminator; the terminator itself is not sent. */
    while (*str) {
        UART1_SendChar(*str++);
    }
}

/* -------------------------------------------------------------------------
 *  UART1_SendInt
 * ------------------------------------------------------------------------- */
void UART1_SendInt(int32_t num) {
    /*
     * Buffer sized for the widest possible output: "-2147483648\0" = 12 chars.
     * Digits are stored smallest-first (reverse order) while decomposing num;
     * a second loop transmits them in correct order by counting down the index.
     */
    char    str[12];
    int     i           = 0;
    bool    is_negative = false;

    /* Edge case: zero cannot be decomposed by the loop below. */
    if (num == 0) {
        UART1_SendChar('0');
        return;
    }

    /* Handle negative input: record sign, then work with the absolute value. */
    if (num < 0) {
        is_negative = true;
        num = -num;
    }

    /* Decompose into digits, least-significant first. */
    while (num > 0) {
        str[i++] = (char)((num % 10) + '0');
        num /= 10;
    }

    /* Append sign character so it appears at the front after reversal. */
    if (is_negative) {
        str[i++] = '-';
    }

    /* Transmit the buffer in reverse to produce the correct digit order. */
    while (i > 0) {
        UART1_SendChar(str[--i]);
    }
}

/* -------------------------------------------------------------------------
 *  UART1_SendFloat
 * ------------------------------------------------------------------------- */
void UART1_SendFloat(float f, uint8_t decimal_places) {

    /* Handle negative values: emit '-' then work with the absolute value. */
    if (f < 0.0f) {
        UART1_SendChar('-');
        f = -f;
    }

    /* Transmit the integer part using the integer formatter. */
    int32_t int_part  = (int32_t)f;
    float   remainder = f - (float)int_part;    /* Isolated fractional part. */
    UART1_SendInt(int_part);

    /* Transmit the fractional part digit by digit. */
    if (decimal_places > 0U) {
        UART1_SendChar('.');

        while (decimal_places > 0U) {
            /*
             * Shift the next decimal digit into the integer position:
             *   e.g., remainder = 0.81 → ×10 = 8.1 → digit = 8.
             * Subtract the extracted digit to isolate the next remainder:
             *   8.1 − 8 = 0.1 (ready for the next iteration).
             * FPU rounding error accumulates for decimal_places > 7;
             * for telemetry use cases (1–4 decimal places) this is negligible.
             */
            remainder      *= 10.0f;
            int32_t digit   = (int32_t)remainder;
            UART1_SendChar((char)(digit + '0'));
            remainder      -= (float)digit;
            decimal_places--;
        }
    }
}

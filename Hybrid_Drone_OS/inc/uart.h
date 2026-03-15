/**
 * @file    uart.h
 * @brief   USART1 transmit driver — bare-metal STM32F411 (CMSIS / C11).
 *
 * Design constraints
 *  - Pure C11 procedural; no C++ constructs, no HAL, no Arduino.
 *  - Only external dependencies: <stdint.h>, "stm32f4xx.h".
 *  - Operates exclusively on USART1 (PA9 = TX, PA10 = RX, AF7).
 *  - All transmit operations are blocking (polling TXE/TC flags).
 *  - Baud rate is derived from SystemCoreClock; SystemClock_Config()
 *    must be called before UART1_Init().
 *  - Floating-point formatting is implemented without libc (no printf/sprintf)
 *    to stay compatible with a bare-metal minimal build.
 *
 * Default configuration applied by UART1_Init()
 *  - GPIO pins  : PA9 (TX), PA10 (RX) — push-pull, no pull, AF7.
 *  - Baud rate  : 115200 bps (BRR = SystemCoreClock / 115200).
 *  - Word length: 8 data bits, 1 stop bit, no parity (8N1).
 *  - Mode       : TX + RX enabled; USART peripheral enabled.
 *
 * @note  CHANGE REQUIRED — Author field: replace the placeholder below with
 *        your full name and/or alias before committing to a shared repository.
 *
 * @author  Joshua (Samir Zambrano)
 * @standard JPL C Coding Standard / MISRA-C:2012 (advisory deviations noted)
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f4xx.h"

/* =========================================================================
 *  PUBLIC API
 * ========================================================================= */

/**
 * @brief  Initialize USART1 at 115200 bps, 8N1, TX + RX.
 *
 *  Sequence:
 *   1. Enable USART1 clock (APB2).
 *   2. Enable GPIOA clock (AHB1).
 *   3. Configure PA9 and PA10 as alternate function push-pull, AF7 (USART1).
 *   4. Set BRR = SystemCoreClock / 115200 for the target baud rate.
 *   5. Enable TE (transmitter), RE (receiver), and UE (USART) in CR1.
 *
 * @note  SystemClock_Config() must be called before UART1_Init() so that
 *        SystemCoreClock reflects the actual 100 MHz SYSCLK.  At 100 MHz
 *        the BRR divisor produces an exact 115200 bps with 0 % error.
 */
void UART1_Init(void);

/**
 * @brief  Transmit a single character over USART1.
 *
 *  Blocks until the transmit data register (TDR) is empty (TXE flag),
 *  then writes @p c to USART1->DR.
 *
 * @param[in]  c  ASCII character to transmit.
 */
void UART1_SendChar(char c);

/**
 * @brief  Transmit a null-terminated string over USART1.
 *
 *  Iterates through @p str and calls UART1_SendChar() for each byte until
 *  the null terminator is reached.  The null terminator is not transmitted.
 *
 * @param[in]  str  Pointer to a null-terminated ASCII string.
 *                  Must not be NULL.
 */
void UART1_SendString(const char *str);

/**
 * @brief  Transmit a signed 32-bit integer as a decimal ASCII string.
 *
 *  Converts @p num to its decimal text representation without using
 *  libc functions.  Handles negative numbers (prefixes '-') and zero.
 *  Maximum output width is 11 characters ("-2147483648").
 *
 * @param[in]  num  Signed 32-bit integer to transmit.
 */
void UART1_SendInt(int32_t num);

/**
 * @brief  Transmit a single-precision float as a decimal ASCII string.
 *
 *  Formats @p f as "[-]<integer_part>.<fractional_part>" without using
 *  libc functions.  The fractional part is extracted by successive
 *  multiplication by 10 and truncation using the Cortex-M4 FPU.
 *
 * @param[in]  f               Single-precision value to transmit.
 *                             Handles negative numbers (prefixes '-').
 * @param[in]  decimal_places  Number of decimal digits to emit after
 *                             the decimal point (0 suppresses the point).
 *                             Values > 7 may accumulate FPU rounding error.
 *
 * @note  CHANGE REQUIRED — If @p decimal_places = 0, the decimal point
 *        character is suppressed but UART1_SendInt() is still called for
 *        the integer part.  Confirm this is the desired behaviour for your
 *        application before production use.
 */
void UART1_SendFloat(float f, uint8_t decimal_places);

#endif /* UART_H */

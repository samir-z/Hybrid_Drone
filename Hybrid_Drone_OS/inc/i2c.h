/**
 * @file    i2c.h
 * @brief   I2C1 register-level driver — bare-metal STM32F411 (CMSIS / C11).
 *
 * Design constraints
 *  - Pure C11 procedural; no C++ constructs, no HAL, no Arduino.
 *  - Only external dependencies: <stdint.h>, "stm32f4xx.h".
 *  - Operates exclusively on I2C1 (PB6 = SCL, PB7 = SDA, AF4).
 *  - All bus operations are blocking with a software timeout guard.
 *  - Fast Mode (400 kHz) enabled; TRISE and CCR values computed for
 *    PCLK1 = 50 MHz (APB1 after /2 prescaler at 100 MHz SYSCLK).
 *
 * Default configuration applied by I2C1_Init()
 *  - GPIO pins  : PB6 (SCL), PB7 (SDA) — open-drain, pull-up, AF4.
 *  - Bus speed  : Fast Mode — 400 kHz (CCR = 5, Fast Mode duty cycle 16/9).
 *  - PCLK1 freq : 50 MHz (written to I2C1->CR2).
 *  - TRISE      : 16 (maximum rise time = 300 ns at 50 MHz  → 300 ns × 50 = 15, +1).
 *
 * Return codes
 *  - 1 : operation succeeded.
 *  - 0 : I2C error (NACK, bus timeout, or arbitration loss); bus left idle.
 *
 * @note  CHANGE REQUIRED — Author field: replace the placeholder below with
 *        your full name and/or alias before committing to a shared repository.
 *
 * @author  Joshua (Samir Zambrano)
 * @standard JPL C Coding Standard / MISRA-C:2012 (advisory deviations noted)
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "stm32f4xx.h"

/* =========================================================================
 *  PUBLIC API
 * ========================================================================= */

/**
 * @brief  Initialize I2C1 peripheral and its GPIO pins.
 *
 *  Sequence:
 *   1. Enable I2C1 and GPIOB clocks via RCC.
 *   2. Assert and release I2C1 reset (APB1RSTR).
 *   3. Configure PB6 and PB7 as open-drain alternate-function (AF4).
 *   4. Enable internal pull-up resistors on PB6 and PB7.
 *   5. Set CR2 = 50 (PCLK1 frequency in MHz).
 *   6. Configure CCR for Fast Mode 400 kHz with 16/9 duty cycle.
 *   7. Set TRISE for Fast Mode.
 *   8. Enable the peripheral via CR1_PE.
 *
 * @note  SystemClock_Config() must be called before I2C1_Init() so that
 *        PCLK1 is already running at 50 MHz when CCR/TRISE are programmed.
 */
void I2C1_Init(void);

/**
 * @brief  Verify that a device ACKs its 7-bit address on the I2C bus.
 *
 *  Generates a START, transmits the address with the write bit, checks for
 *  an ACK, then issues a STOP.  No data bytes are transferred.
 *  Useful as a presence check before a full read/write cycle.
 *
 * @param[in]  address  7-bit I2C slave address (not shifted; e.g., 0x68).
 * @retval     1        Device acknowledged — present and operational.
 * @retval     0        No ACK received, or bus timeout occurred.
 */
uint8_t I2C1_Ping(uint8_t address);

/**
 * @brief  Write a single byte to a slave register.
 *
 *  Protocol sequence:
 *    [S][ADDR+W][ACK][reg][ACK][data][ACK][P]
 *
 * @param[in]  address  7-bit I2C slave address (e.g., 0x68).
 * @param[in]  reg      Target register address on the slave device.
 * @param[in]  data     Byte to write into the register.
 * @retval     1        Register written successfully.
 * @retval     0        I2C error (NACK or timeout); register state undefined.
 */
uint8_t I2C1_Write_Reg(uint8_t address, uint8_t reg, uint8_t data);

/**
 * @brief  Read a single byte from a slave register.
 *
 *  Protocol sequence:
 *    [S][ADDR+W][ACK][reg][ACK][Sr][ADDR+R][ACK][DATA][NACK][P]
 *
 *  The NACK before STOP is mandatory for single-byte reception per the
 *  STM32F4 I2C peripheral reference manual (RM0383, §22.3.3).
 *
 * @param[in]  address  7-bit I2C slave address.
 * @param[in]  reg      Register address to read from.
 * @retval     uint8_t  Byte read from the slave register.
 * @retval     0        Returned on timeout — ambiguous if register value is 0;
 *                      use I2C1_Ping() first to confirm device presence.
 */
uint8_t I2C1_Read_Reg(uint8_t address, uint8_t reg);

/**
 * @brief  Read a contiguous block of registers via I2C repeated-START.
 *
 *  Protocol sequence:
 *    [S][ADDR+W][ACK][reg][ACK]
 *    [Sr][ADDR+R][ACK]
 *    [DATA_0][ACK] ... [DATA_n-2][ACK] [DATA_n-1][NACK][P]
 *
 *  The STM32F4 I2C hardware requires special ACK/STOP ordering for the
 *  last two bytes: ACK is disabled and STOP is pre-programmed before
 *  reading the penultimate byte to guarantee correct NACK on the final byte.
 *
 * @param[in]  address  7-bit I2C slave address.
 * @param[in]  reg      First register address to read (auto-incremented by slave).
 * @param[out] buf      Caller-allocated buffer of at least @p length bytes.
 *                      Must not be NULL.
 * @param[in]  length   Number of consecutive bytes to read (1–255).
 * @retval     1        All @p length bytes received without error.
 * @retval     0        Bus error or timeout; contents of @p buf are undefined.
 */
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t length);

#endif /* I2C_H */

/**
 * @file    main.h
 * @brief   Project-wide interface header for Hybrid_Drone_OS.
 *
 * Exposes the peripheral API implemented in main.c so that driver modules
 * (e.g. mpu6050.c) can reference these functions without including the full
 * main.c translation unit.
 *
 * Include order expected by driver files:
 *   #include <stdint.h>
 *   #include <stdbool.h>
 *   #include "main.h"
 *
 * @standard JPL C Coding Standard / MISRA-C:2012
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/* =========================================================================
 *  TIMING
 * ========================================================================= */

/**
 * @brief  Blocking millisecond delay using the SysTick counter.
 * @param  ms  Number of milliseconds to wait.
 */
void delay_ms(uint32_t ms);

/* =========================================================================
 *  I2C1 PERIPHERAL PRIMITIVES  (PB6 = SCL, PB7 = SDA, 100 kHz, 7-bit addr)
 * ========================================================================= */

/**
 * @brief  Probe whether a device ACKs its address on the I2C bus.
 * @param  address  7-bit slave address.
 * @retval 1  — device present and acknowledges.
 * @retval 0  — NACK, bus error, or timeout.
 */
uint8_t I2C1_Ping(uint8_t address);

/**
 * @brief  Write one byte to a device register.
 * @param  address  7-bit slave address.
 * @param  reg      Target register address.
 * @param  data     Byte value to write.
 * @retval 1  — write confirmed.
 * @retval 0  — NACK, bus error, or timeout.
 */
uint8_t I2C1_Write_Reg(uint8_t address, uint8_t reg, uint8_t data);

/**
 * @brief  Read one byte from a device register (uses repeated-START).
 * @param  address  7-bit slave address.
 * @param  reg      Source register address.
 * @retval Byte read from the register.
 *         Returns 0 on timeout or bus error; call I2C1_Ping() to distinguish
 *         genuine 0x00 data from a communication failure.
 */
uint8_t I2C1_Read_Reg(uint8_t address, uint8_t reg);

/**
 * @brief  Read a contiguous block of registers via I2C repeated-START.
 *
 *  Protocol: [S][ADDR+W][ACK][reg][ACK][Sr][ADDR+R][ACK]
 *            [D0][ACK] ... [Dn-2][ACK] [Dn-1][NACK][P]
 *
 *  @note  Implementation pending — to be added in main.c alongside the
 *         other I2C1 primitives.
 *
 * @param  address  7-bit slave address.
 * @param  reg      First register address in the sequential block.
 * @param  buf      Caller-provided buffer of at least 'length' bytes.
 * @param  length   Number of bytes to read (1–255).
 * @retval 1  — all bytes received.
 * @retval 0  — bus error or timeout; buf contents are undefined.
 */
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg,
                         uint8_t *buf, uint8_t length);

#endif /* MAIN_H */

/**
 * @file    i2c.c
 * @brief   Implementation of the I2C1 register-level driver — STM32F411.
 *
 * Implementation notes
 *  - All blocking waits are guarded by a 10 000-iteration software timeout.
 *    This prevents the CPU from hanging indefinitely if the bus is stuck
 *    (e.g., a slave holding SDA LOW after a power-on glitch).
 *  - The ADDR flag on the STM32F4 I2C peripheral is cleared by performing
 *    a sequential read of SR1 followed by SR2 (RM0383 §22.3.3).  The
 *    (void) casts are intentional — the values are discarded; the side
 *    effect (clearing ADDR) is what matters.
 *  - CCR encoding for Fast Mode with DUTY = 16/9:
 *      Bit 15 (F/S)  = 1 → Fast Mode (400 kHz).
 *      Bit 14 (DUTY) = 1 → 16/9 Tlow/Thigh duty cycle.
 *      Bits[11:0]    = 5 → CCR value.
 *    Period = (9 + 16) × CCR / PCLK1 = 25 × 5 / 50 MHz = 2.5 µs → 400 kHz.
 *  - TRISE for Fast Mode: TRISE = floor(T_rise_max × PCLK1) + 1
 *                                = floor(300 ns × 50 MHz) + 1
 *                                = floor(15) + 1 = 16.
 */

#include "i2c.h"

/* =========================================================================
 *  FUNCTION IMPLEMENTATIONS
 * ========================================================================= */

/* -------------------------------------------------------------------------
 *  I2C1_Init
 * ------------------------------------------------------------------------- */
void I2C1_Init(void) {

    /* --- 1. Enable peripheral clocks ------------------------------------- */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;    /* I2C1 bus clock (APB1).        */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   /* GPIOB GPIO clock (AHB1).      */

    /* --- 2. Reset I2C1 to clear any stale configuration ------------------ */
    /*
     * Asserting and immediately de-asserting the reset ensures the peripheral
     * starts from a known state, which is particularly important after a
     * warm reset where the I2C clock may have been left mid-transaction.
     */
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* --- 3. Configure PB6 (SCL) and PB7 (SDA) as AF4, open-drain -------- */
    /*
     * MODER[13:12] = 10 (AF mode) for PB6.
     * MODER[15:14] = 10 (AF mode) for PB7.
     */
    GPIOB->MODER &= ~((3U << 12) | (3U << 14));
    GPIOB->MODER |=  (2U << 12) | (2U << 14);

    /*
     * OTYPER bits 6 and 7 = 1 → open-drain.
     * I2C requires open-drain to allow multiple masters and slaves to
     * drive the bus LOW without short-circuiting a pushed-HIGH line.
     */
    GPIOB->OTYPER |= (1U << 6) | (1U << 7);

    /*
     * PUPDR[13:12] = 01 (pull-up) for PB6.
     * PUPDR[15:14] = 01 (pull-up) for PB7.
     * Internal pull-ups bias idle SCL and SDA HIGH.  For 400 kHz and board
     * trace capacitance > 50 pF, external pull-up resistors (e.g., 2.2 kΩ)
     * are recommended in addition to these weak internals (~40 kΩ).
     */
    GPIOB->PUPDR &= ~((3U << 12) | (3U << 14));
    GPIOB->PUPDR |=  (1U << 12) | (1U << 14);

    /*
     * AFR[0] controls pins 0–7.
     * AF4 is the I2C1/2/3 alternate function for PB6 and PB7 (RM0383 Table 9).
     * Bits [27:24] = 0100 for PB6; bits [31:28] = 0100 for PB7.
     */
    GPIOB->AFR[0] &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->AFR[0] |=  (4U << 24) | (4U << 28);

    /* --- 4. Configure I2C1 peripheral ------------------------------------ */
    I2C1->CR2   = 50U;                          /* PCLK1 in MHz (50 MHz).    */
    I2C1->CCR   = (1U << 15)                    /* F/S = 1: Fast Mode.       */
                | (1U << 14)                    /* DUTY = 1: 16/9 ratio.     */
                | 5U;                           /* CCR = 5 → 400 kHz.        */
    I2C1->TRISE = 16U;                          /* Max rise time for FM.     */
    I2C1->CR1   = I2C_CR1_PE;                   /* Enable I2C1 peripheral.   */
}

/* -------------------------------------------------------------------------
 *  I2C1_Ping
 * ------------------------------------------------------------------------- */
uint8_t I2C1_Ping(uint8_t address) {
    uint32_t timeout = 10000U;

    /* Generate START condition and wait for the SB (Start Bit) flag. */
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
    }

    /* Transmit 7-bit address with write bit (bit 0 = 0). */
    I2C1->DR = (address << 1U) | 0U;
    timeout = 10000U;

    /*
     * ADDR flag is set once the address byte has been transmitted and an
     * ACK has been received.  If the slave is absent, the AF (Acknowledge
     * Failure) flag is set instead — exit immediately with STOP.
     */
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
    }

    /* Clear ADDR flag (hardware requirement: read SR1 then SR2). */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

/* -------------------------------------------------------------------------
 *  I2C1_Write_Reg
 * ------------------------------------------------------------------------- */
uint8_t I2C1_Write_Reg(uint8_t address, uint8_t reg, uint8_t data) {
    uint32_t timeout;

    /* --- START ----------------------------------------------------------- */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) return 0U;
    }

    /* --- Address phase (write direction) --------------------------------- */
    I2C1->DR = (address << 1U) | 0U;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }
    (void)I2C1->SR1;    /* Clear ADDR flag. */
    (void)I2C1->SR2;

    /* --- Register address byte ------------------------------------------- */
    /*
     * TXE (Transmit-data-register Empty) is set once the byte in DR has
     * been transferred to the shift register.  Checking AF here lets us
     * abort early if the slave NACKs the register pointer.
     */
    I2C1->DR = reg;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /* --- Data byte ------------------------------------------------------- */
    I2C1->DR = data;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /* --- STOP ------------------------------------------------------------ */
    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

/* -------------------------------------------------------------------------
 *  I2C1_Read_Reg
 * ------------------------------------------------------------------------- */
uint8_t I2C1_Read_Reg(uint8_t address, uint8_t reg) {
    uint32_t timeout;

    /* --- START + address (write) to set register pointer ----------------- */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) return 0U;
    }

    I2C1->DR = (address << 1U) | 0U;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    /* --- Send register address ------------------------------------------- */
    I2C1->DR = reg;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /* --- Repeated START + address (read direction) ----------------------- */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) return 0U;
    }

    I2C1->DR = (address << 1U) | 1U;    /* Bit 0 = 1: read direction.        */
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /*
     * Single-byte reception sequence (RM0383 §22.3.3, "Transfer sequence
     * diagram for master receiver — N = 1"):
     *  1. Disable ACK BEFORE clearing ADDR (prevents the hardware from
     *     sending a spurious ACK on the incoming byte).
     *  2. Clear ADDR flag (SR1 + SR2 read).
     *  3. Program STOP immediately after — hardware generates STOP after
     *     the shift register has captured the byte.
     *  4. Wait for RXNE, then read DR.
     */
    I2C1->CR1 &= ~I2C_CR1_ACK;     /* Disable ACK — NACK the single byte.   */
    (void)I2C1->SR1;                /* Clear ADDR flag (step 1 of 2).        */
    (void)I2C1->SR2;                /* Clear ADDR flag (step 2 of 2).        */
    I2C1->CR1 |= I2C_CR1_STOP;     /* Schedule STOP after byte is received. */

    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
        if (--timeout == 0U) return 0U;
    }
    return (uint8_t)I2C1->DR;
}

/* -------------------------------------------------------------------------
 *  I2C1_Read_Burst
 * ------------------------------------------------------------------------- */
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t length) {
    uint32_t timeout;

    /* --- START + address (write) to set register pointer ----------------- */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) return 0U;
    }

    I2C1->DR = (address << 1U) | 0U;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    /* --- Send register start address ------------------------------------- */
    I2C1->DR = reg;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /* --- Repeated START + address (read direction) ----------------------- */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0U) return 0U;
    }

    I2C1->DR = (address << 1U) | 1U;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
        if (--timeout == 0U) return 0U;
    }

    /* --- Reception phase ------------------------------------------------- */
    if (length == 1U) {
        /*
         * N=1 special case (identical to I2C1_Read_Reg):
         * Disable ACK BEFORE clearing ADDR, then program STOP.
         */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;

        timeout = 10000U;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--timeout == 0U) return 0U;
        }
        buf[0] = (uint8_t)I2C1->DR;

    } else {
        /*
         * N > 1 case — ACK is sent for bytes 0 .. N-3.
         * For byte N-2 (second-to-last): disable ACK and program STOP
         * before reading, so the hardware generates NACK + STOP after
         * the last byte is shifted in (RM0383 §22.3.3, N>2 diagram).
         */
        I2C1->CR1 |= I2C_CR1_ACK;
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        for (uint8_t i = 0U; i < length; i++) {
            if (i == (uint8_t)(length - 2U)) {
                /* Pre-programme NACK+STOP for byte N-2, effective after N-1. */
                timeout = 10000U;
                while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
                    if (--timeout == 0U) return 0U;
                }
                I2C1->CR1 &= ~I2C_CR1_ACK;     /* NACK the next (last) byte. */
                I2C1->CR1 |= I2C_CR1_STOP;     /* STOP after the last byte.  */
                buf[i] = (uint8_t)I2C1->DR;

            } else {
                /* All other bytes — wait for RXNE and read normally. */
                timeout = 10000U;
                while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
                    if (--timeout == 0U) return 0U;
                }
                buf[i] = (uint8_t)I2C1->DR;
            }
        }
    }

    return 1U;
}

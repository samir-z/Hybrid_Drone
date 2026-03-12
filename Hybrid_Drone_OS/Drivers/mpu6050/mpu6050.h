/**
 * @file    mpu6050.h
 * @brief   MPU-6050 6-axis IMU driver — bare-metal STM32F411 (CMSIS / C11).
 *
 * Design constraints
 *  - Pure C11 procedural; no C++ constructs, no HAL, no Arduino.
 *  - Only external dependencies: <stdint.h>, <stdbool.h>, "main.h".
 *  - All raw offsets are compile-time #defines (computed externally by MATLAB).
 *  - Scaling uses the Cortex-M4 FPU (fpv4-sp-d16, hard-ABI).
 *  - I2C access exclusively via the three validated register-level primitives
 *    in main.c plus the burst-read stub declared below.
 *
 * Default configuration applied by MPU6050_Init()
 *  - Gyroscope  : ±500 °/s  → FS_SEL  = 1, sensitivity = 65.5 LSB/(°/s)
 *  - Accelerometer : ±4 g   → AFS_SEL = 1, sensitivity = 8192 LSB/g
 *  - DLPF bandwidth: 42 Hz  → DLPF_CFG = 3 (suitable for multirotor)
 *  - Clock source  : internal 8 MHz oscillator (CLKSEL = 0)
 *
 * Conventions
 *  - Return values : MPU6050_OK (1) / MPU6050_ERR (0)
 *  - Physical units: acceleration in m/s², angular rate in °/s, temperature in °C.
 *
 * @author  MechaCore Firmware Team
 * @standard JPL C Coding Standard / MISRA-C:2012 (advisory deviations noted)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   /* I2C1 primitives: Ping, Write_Reg, Read_Reg */

/* =========================================================================
 *  DEVICE ADDRESS
 * ========================================================================= */

/** 7-bit I2C address when AD0 pin is pulled to GND. */
#define MPU6050_I2C_ADDR        (0x68U)

/* =========================================================================
 *  REGISTER MAP  (MPU-6050 Product Specification, Rev 3.4)
 * ========================================================================= */

#define MPU6050_REG_CONFIG      (0x1AU)  /**< DLPF and frame sync config          */
#define MPU6050_REG_GYRO_CFG    (0x1BU)  /**< Gyroscope full-scale range           */
#define MPU6050_REG_ACCEL_CFG   (0x1CU)  /**< Accelerometer full-scale range       */
#define MPU6050_REG_ACCEL_XOUT_H (0x3BU) /**< Burst start: Accel X MSB             */
#define MPU6050_REG_PWR_MGMT_1  (0x6BU)  /**< Power management; SLEEP bit = bit 6  */
#define MPU6050_REG_WHO_AM_I    (0x75U)  /**< Device identity register             */

/** Expected WHO_AM_I response — validates device presence on the bus. */
#define MPU6050_WHO_AM_I_VAL    (0x68U)

/* =========================================================================
 *  REGISTER FIELD VALUES
 * ========================================================================= */

/**
 * PWR_MGMT_1 — write 0x00 to clear SLEEP bit and select internal 8 MHz clock.
 */
#define MPU6050_PWR_WAKE        (0x00U)

/**
 * GYRO_CONFIG — FS_SEL field occupies bits [4:3].
 *  0b00001000 → FS_SEL = 1 → ±500 °/s
 */
#define MPU6050_GYRO_FS_500DPS  (0x08U)

/**
 * ACCEL_CONFIG — AFS_SEL field occupies bits [4:3].
 *  0b00001000 → AFS_SEL = 1 → ±4 g
 */
#define MPU6050_ACCEL_FS_4G     (0x08U)

/* =========================================================================
 *  SCALE FACTORS  (single-precision, used by Cortex-M4 FPU)
 * ========================================================================= */

/** Accelerometer sensitivity for the ±4 g range: 8192 LSB per g. */
#define MPU6050_ACCEL_SENS      (8192.0f)

/** Gravitational acceleration constant [m/s²]. */
#define MPU6050_G_MS2           (9.80665f)

/** Gyroscope sensitivity for the ±500 °/s range: 65.5 LSB per °/s. */
#define MPU6050_GYRO_SENS       (65.5f)

/* =========================================================================
 *  DLPF DEFAULT
 * ========================================================================= */

/**
 * DLPF_CFG = 3 → Accelerometer BW ≈ 44 Hz, Gyroscope BW ≈ 42 Hz.
 * Write into CONFIG register bits [2:0].
 * Appropriate for a multirotor flight controller attitude loop.
 */
#define MPU6050_DLPF_DEFAULT    (3U)

/* =========================================================================
 *  RAW OFFSETS  (replace with MATLAB-computed values before production build)
 *
 *  Units: raw ADC counts (int16_t).
 *  Subtracted from raw readings BEFORE FPU conversion in MPU6050_Scale().
 *  Sign convention: positive offset shifts sensor reading DOWN by that amount.
 * ========================================================================= */

#define MPU6050_OFFSET_AX       ( 0)   /**< Accelerometer X offset [LSB] */
#define MPU6050_OFFSET_AY       ( 0)   /**< Accelerometer Y offset [LSB] */
#define MPU6050_OFFSET_AZ       ( 0)   /**< Accelerometer Z offset [LSB] */
#define MPU6050_OFFSET_GX       ( 0)   /**< Gyroscope X offset [LSB]     */
#define MPU6050_OFFSET_GY       ( 0)   /**< Gyroscope Y offset [LSB]     */
#define MPU6050_OFFSET_GZ       ( 0)   /**< Gyroscope Z offset [LSB]     */

/* =========================================================================
 *  RETURN CODES
 * ========================================================================= */

#define MPU6050_OK              (1U)   /**< Operation completed successfully. */
#define MPU6050_ERR             (0U)   /**< I2C failure or invalid identity.  */

/* =========================================================================
 *  DATA STRUCTURE
 * ========================================================================= */

/**
 * @brief  Complete IMU measurement snapshot.
 *
 * Workflow:
 *  1. Call MPU6050_Read()  → populates the *_raw fields.
 *  2. Call MPU6050_Scale() → populates the scaled physical-unit fields,
 *                            using the *_raw values offset-corrected in int16_t
 *                            arithmetic before any float conversion.
 */
typedef struct
{
    /* --- Raw ADC output registers (two's complement, big-endian assembled) --- */
    int16_t accel_x_raw;   /**< Accelerometer X [LSB]          */
    int16_t accel_y_raw;   /**< Accelerometer Y [LSB]          */
    int16_t accel_z_raw;   /**< Accelerometer Z [LSB]          */
    int16_t temp_raw;      /**< Temperature sensor [LSB]       */
    int16_t gyro_x_raw;    /**< Gyroscope X [LSB]              */
    int16_t gyro_y_raw;    /**< Gyroscope Y [LSB]              */
    int16_t gyro_z_raw;    /**< Gyroscope Z [LSB]              */

    /* --- Physical values (FPU computed, offset-corrected) --- */
    float accel_x_ms2;     /**< Accelerometer X [m/s²]         */
    float accel_y_ms2;     /**< Accelerometer Y [m/s²]         */
    float accel_z_ms2;     /**< Accelerometer Z [m/s²]         */
    float temp_degC;       /**< Die temperature [°C]           */
    float gyro_x_dps;      /**< Gyroscope X [°/s]              */
    float gyro_y_dps;      /**< Gyroscope Y [°/s]              */
    float gyro_z_dps;      /**< Gyroscope Z [°/s]              */

} MPU6050_Data_t;

/* =========================================================================
 *  I2C BURST-READ DECLARATION
 *  (implementation deferred — to be written in main.c alongside
 *   I2C1_Ping / I2C1_Write_Reg / I2C1_Read_Reg)
 * ========================================================================= */

/**
 * @brief  Read a contiguous block of 'length' registers via I2C repeated-START.
 *
 *  Protocol sequence:
 *    [S][ADDR+W][ACK][reg][ACK]
 *    [Sr][ADDR+R][ACK]
 *    [DATA_0][ACK] ... [DATA_n-2][ACK] [DATA_n-1][NACK][P]
 *
 * @param[in]  address  7-bit I2C slave address.
 * @param[in]  reg      First register address to read.
 * @param[out] buf      Destination buffer; caller must provide at least
 *                      'length' bytes of valid writable memory.
 * @param[in]  length   Number of consecutive bytes to read (1–255).
 * @retval     MPU6050_OK  (1) — all bytes received without error.
 * @retval     MPU6050_ERR (0) — bus error or timeout; buf contents undefined.
 */
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg,
                         uint8_t *buf, uint8_t length);

/* =========================================================================
 *  PUBLIC API
 * ========================================================================= */

/**
 * @brief  Initialize the MPU-6050.
 *
 *  Sequence:
 *   1. Write 0x00 to PWR_MGMT_1 → clears SLEEP, internal 8 MHz clock.
 *   2. I2C1_Ping() to confirm device ACKs on the bus.
 *   3. Read WHO_AM_I → verify device identity (0x68).
 *   4. Write GYRO_CONFIG  = 0x08 → ±500 °/s.
 *   5. Write ACCEL_CONFIG = 0x08 → ±4 g.
 *   6. Write CONFIG[2:0]  = DLPF_DEFAULT → 42 Hz bandwidth.
 *
 * @retval MPU6050_OK  (1) — sensor ready.
 * @retval MPU6050_ERR (0) — I2C failure or wrong WHO_AM_I; caller must halt.
 */
uint8_t MPU6050_Init(void);

/**
 * @brief  Read all inertial sensor registers in one 14-byte burst.
 *
 *  Populates only the *_raw fields of *data_out.
 *  Call MPU6050_Scale() afterwards to obtain physical values.
 *
 * @param[out] data_out  Pointer to the destination MPU6050_Data_t struct.
 *                       Must not be a null pointer.
 * @retval MPU6050_OK  (1) — raw fields updated.
 * @retval MPU6050_ERR (0) — I2C burst failed; struct contents unchanged.
 */
uint8_t MPU6050_Read(MPU6050_Data_t *data_out);

/**
 * @brief  Convert raw ADC counts to physical units using the Cortex-M4 FPU.
 *
 *  The integer offsets (#define MPU6050_OFFSET_*) are subtracted in int16_t
 *  arithmetic first, preventing float-domain accumulation error.  The FPU
 *  then performs a single FDIV + FMUL sequence per axis.
 *
 *  MPU6050_Read() must have been called successfully before this function.
 *
 * @param[in,out] data  Pointer to a populated MPU6050_Data_t.
 *                      Raw fields are read; scaled fields are written.
 *                      Must not be a null pointer.
 */
void MPU6050_Scale(MPU6050_Data_t *data);

/**
 * @brief  Configure the Digital Low-Pass Filter (CONFIG register, bits [2:0]).
 *
 *  DLPF_CFG table (MPU-6050 datasheet, Table 13):
 *   0 → Accel 260 Hz / Gyro 256 Hz  (DLPF effectively bypassed)
 *   1 → Accel 184 Hz / Gyro 188 Hz
 *   2 → Accel  94 Hz / Gyro  98 Hz
 *   3 → Accel  44 Hz / Gyro  42 Hz  ← MPU6050_DLPF_DEFAULT
 *   4 → Accel  21 Hz / Gyro  20 Hz
 *   5 → Accel  10 Hz / Gyro  10 Hz
 *   6 → Accel   5 Hz / Gyro   5 Hz
 *   7 → Reserved
 *
 * @param[in]  dlpf_cfg  Value in range [0, 6]. Values > 6 are silently masked
 *                       to bits [2:0] and written; the EXT_SYNC_SET field is
 *                       forced to 0.
 * @retval MPU6050_OK  (1) — register updated.
 * @retval MPU6050_ERR (0) — I2C write failed.
 */
uint8_t MPU6050_SetDLPF(uint8_t dlpf_cfg);

#endif /* MPU6050_H */

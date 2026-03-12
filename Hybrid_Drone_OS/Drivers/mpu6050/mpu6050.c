/**
 * @file    mpu6050.c
 * @brief   MPU-6050 6-axis IMU driver implementation — bare-metal STM32F411.
 *
 * All I2C transactions are delegated exclusively to the four bus primitives
 * declared in main.h / mpu6050.h.  No direct register access to I2C1 occurs
 * in this file; this preserves the layered architecture.
 *
 * FPU usage: the Cortex-M4 FPU (fpv4-sp-d16, -mfloat-abi=hard) is enabled by
 * SystemClock_Config() in main.c before this driver is ever called.  All float
 * arithmetic in MPU6050_Scale() compiles to VFP single-precision instructions
 * (VCVT, VDIV, VMUL) with zero software-emulation overhead.
 *
 * @author  MechaCore Firmware Team
 * @standard JPL C Coding Standard / MISRA-C:2012
 */

#include "mpu6050.h"

/* =========================================================================
 *  INTERNAL HELPERS  (translation-unit scope only)
 * ========================================================================= */

/**
 * @brief  Assemble a signed 16-bit two's-complement integer from two raw bytes.
 *
 *  The MPU-6050 stores all outputs big-endian (MSB first).
 *  This helper avoids any pointer aliasing or `memcpy` dependency.
 *
 * @param  hi  High byte (register N,   e.g. ACCEL_XOUT_H).
 * @param  lo  Low  byte (register N+1, e.g. ACCEL_XOUT_L).
 * @retval Signed 16-bit integer value.
 */
static inline int16_t s_assemble_int16(uint8_t hi, uint8_t lo)
{
    /* Cast to uint16_t before shifting to avoid implementation-defined
     * behaviour on signed left-shift (MISRA-C:2012 Rule 12.2). */
    return (int16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);
}

/* =========================================================================
 *  PUBLIC API IMPLEMENTATIONS
 * ========================================================================= */

/* -------------------------------------------------------------------------
 *  MPU6050_Init
 * ---------------------------------------------------------------------- */
uint8_t MPU6050_Init(void)
{
    /*
     * Step 1 — Exit sleep mode.
     *
     * PWR_MGMT_1 (0x6B):
     *   Bit 6   SLEEP  = 0  → device active
     *   Bits[2:0] CLKSEL = 0 → internal 8 MHz RC oscillator
     *
     * The MPU-6050 powers up with SLEEP = 1; writing 0x00 is the mandatory
     * first transaction.
     */
    if (I2C1_Write_Reg(MPU6050_I2C_ADDR,
                       MPU6050_REG_PWR_MGMT_1,
                       MPU6050_PWR_WAKE) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    /*
     * Step 2 — Confirm device presence.
     *
     * I2C1_Ping() generates a bare START+ADDR+STOP to verify the slave ACKs
     * its address.  A NACK here indicates wiring or address mismatch.
     */
    if (I2C1_Ping(MPU6050_I2C_ADDR) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    /*
     * Step 3 — Validate device identity.
     *
     * WHO_AM_I (0x75) returns the upper 6 bits of the 7-bit I2C address,
     * which for the MPU-6050 is always 0x68.  A wrong value indicates a
     * different device at this address (e.g. MPU-6500 returns 0x70).
     */
    uint8_t who_am_i = I2C1_Read_Reg(MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I);
    if (who_am_i != MPU6050_WHO_AM_I_VAL)
    {
        return MPU6050_ERR;
    }

    /*
     * Step 4 — Configure gyroscope full-scale range.
     *
     * GYRO_CONFIG (0x1B):
     *   Bits [4:3] FS_SEL = 0b01 → ±500 °/s, sensitivity = 65.5 LSB/(°/s)
     *   0x08 = 0b00001000 → FS_SEL = 1, all other bits cleared (no self-test).
     */
    if (I2C1_Write_Reg(MPU6050_I2C_ADDR,
                       MPU6050_REG_GYRO_CFG,
                       MPU6050_GYRO_FS_500DPS) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    /*
     * Step 5 — Configure accelerometer full-scale range.
     *
     * ACCEL_CONFIG (0x1C):
     *   Bits [4:3] AFS_SEL = 0b01 → ±4 g, sensitivity = 8192 LSB/g
     *   0x08 = 0b00001000 → AFS_SEL = 1, no self-test.
     */
    if (I2C1_Write_Reg(MPU6050_I2C_ADDR,
                       MPU6050_REG_ACCEL_CFG,
                       MPU6050_ACCEL_FS_4G) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    /*
     * Step 6 — Activate the Digital Low-Pass Filter.
     *
     * Delegates to MPU6050_SetDLPF() to avoid duplicating the mask logic.
     * Default configuration: DLPF_CFG = 3 → 42 Hz gyro / 44 Hz accel BW.
     */
    if (MPU6050_SetDLPF(MPU6050_DLPF_DEFAULT) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    return MPU6050_OK;
}

/* -------------------------------------------------------------------------
 *  MPU6050_Read
 * ---------------------------------------------------------------------- */
uint8_t MPU6050_Read(MPU6050_Data_t *data_out)
{
    uint8_t buf[14U];

    /* Guard against null pointer before any dereference (MISRA-C:2012 D4.1). */
    if (data_out == (MPU6050_Data_t *)0U)
    {
        return MPU6050_ERR;
    }

    /*
     * Single burst transaction starting at ACCEL_XOUT_H (0x3B).
     *
     * The MPU-6050 auto-increments the internal register pointer, so 14
     * consecutive reads yield:
     *   buf[ 0.. 1] = ACCEL_XOUT_H/L
     *   buf[ 2.. 3] = ACCEL_YOUT_H/L
     *   buf[ 4.. 5] = ACCEL_ZOUT_H/L
     *   buf[ 6.. 7] = TEMP_OUT_H/L
     *   buf[ 8.. 9] = GYRO_XOUT_H/L
     *   buf[10..11] = GYRO_YOUT_H/L
     *   buf[12..13] = GYRO_ZOUT_H/L
     *
     * This minimises bus transactions and guarantees a consistent snapshot
     * of all axes from the same internal sample instant.
     */
    if (I2C1_Read_Burst(MPU6050_I2C_ADDR,
                        MPU6050_REG_ACCEL_XOUT_H,
                        buf,
                        14U) == MPU6050_ERR)
    {
        return MPU6050_ERR;
    }

    /* Assemble signed 16-bit integers from big-endian byte pairs. */
    data_out->accel_x_raw = s_assemble_int16(buf[0U],  buf[1U]);
    data_out->accel_y_raw = s_assemble_int16(buf[2U],  buf[3U]);
    data_out->accel_z_raw = s_assemble_int16(buf[4U],  buf[5U]);
    data_out->temp_raw    = s_assemble_int16(buf[6U],  buf[7U]);
    data_out->gyro_x_raw  = s_assemble_int16(buf[8U],  buf[9U]);
    data_out->gyro_y_raw  = s_assemble_int16(buf[10U], buf[11U]);
    data_out->gyro_z_raw  = s_assemble_int16(buf[12U], buf[13U]);

    return MPU6050_OK;
}

/* -------------------------------------------------------------------------
 *  MPU6050_Scale
 * ---------------------------------------------------------------------- */
void MPU6050_Scale(MPU6050_Data_t *data)
{
    if (data == (MPU6050_Data_t *)0U)
    {
        return;
    }

    /*
     * Offset subtraction in integer domain.
     *
     * Performing the subtraction before float conversion avoids two sources
     * of error:
     *  a) FP representation error in the offset itself (integer → float cast).
     *  b) Catastrophic cancellation when a large raw value cancels a large
     *     float offset near the limits of single-precision representability.
     *
     * The result is a signed 16-bit integer difference; no overflow is
     * possible for reasonable (MATLAB-calibrated) offsets within ±32767 LSB.
     */
    int16_t ax_c = data->accel_x_raw - (int16_t)(MPU6050_OFFSET_AX);
    int16_t ay_c = data->accel_y_raw - (int16_t)(MPU6050_OFFSET_AY);
    int16_t az_c = data->accel_z_raw - (int16_t)(MPU6050_OFFSET_AZ);
    int16_t gx_c = data->gyro_x_raw  - (int16_t)(MPU6050_OFFSET_GX);
    int16_t gy_c = data->gyro_y_raw  - (int16_t)(MPU6050_OFFSET_GY);
    int16_t gz_c = data->gyro_z_raw  - (int16_t)(MPU6050_OFFSET_GZ);

    /*
     * FPU conversion — Cortex-M4 VFP instructions:
     *
     * Accelerometer:
     *   a [m/s²] = (raw_counts / sensitivity_LSB_per_g) * g
     *            = (raw_counts / 8192.0f) * 9.80665f
     *   The compiler folds the two constants into a single FP literal,
     *   so only one FMUL is emitted per axis.
     *
     * Gyroscope:
     *   ω [°/s] = raw_counts / sensitivity_LSB_per_dps
     *           = raw_counts / 65.5f
     *   One VDIV per axis (or FMUL with precomputed reciprocal depending on -O).
     *
     * Temperature (MPU-6050 Register Map, Rev 4.2, §4.18):
     *   T [°C] = TEMP_OUT / 340.0f + 36.53f
     *   Note: temp_raw is NOT offset-corrected (factory-calibrated sensor).
     */
    data->accel_x_ms2 = ((float)ax_c / MPU6050_ACCEL_SENS) * MPU6050_G_MS2;
    data->accel_y_ms2 = ((float)ay_c / MPU6050_ACCEL_SENS) * MPU6050_G_MS2;
    data->accel_z_ms2 = ((float)az_c / MPU6050_ACCEL_SENS) * MPU6050_G_MS2;

    data->temp_degC   = ((float)data->temp_raw / 340.0f) + 36.53f;

    data->gyro_x_dps  = (float)gx_c / MPU6050_GYRO_SENS;
    data->gyro_y_dps  = (float)gy_c / MPU6050_GYRO_SENS;
    data->gyro_z_dps  = (float)gz_c / MPU6050_GYRO_SENS;
}

/* -------------------------------------------------------------------------
 *  MPU6050_SetDLPF
 * ---------------------------------------------------------------------- */
uint8_t MPU6050_SetDLPF(uint8_t dlpf_cfg)
{
    /*
     * CONFIG register (0x1A) layout:
     *   Bits [7:6] = 0 (reserved)
     *   Bits [5:3] = EXT_SYNC_SET → forced to 0 (no external sync)
     *   Bits [2:0] = DLPF_CFG    → the only field we touch
     *
     * Masking with 0x07U ensures the upper bits are always zero regardless
     * of the caller's argument.
     */
    uint8_t reg_val = dlpf_cfg & 0x07U;

    return I2C1_Write_Reg(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, reg_val);
}

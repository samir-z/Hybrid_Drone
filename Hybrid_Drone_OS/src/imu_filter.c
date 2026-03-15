#include "imu_filter.h"
#include <math.h>

void CF_Init(CF_State_t *cf, float alpha) {
    cf->pitch = 0.0f;
    cf->roll = 0.0f;
    cf->alpha = alpha;
    cf->last_time_ms = 0;
}

void CF_Update(CF_State_t *cf, MPU6050_Data_t *imu, uint32_t current_time_ms) {
    // Calculate time delta in seconds
    float dt = (current_time_ms - cf->last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f; // Handle edge cases (e.g., first update or timer wraparound)    
    cf->last_time_ms = current_time_ms;

    // Integrate gyro rates to get angles
    float pitch_gyro = cf->pitch + imu->gyro_x_dps * dt;
    float roll_gyro = cf->roll + imu->gyro_y_dps * dt;

    // Calculate accelerometer angles (in degrees)
    float accel_pitch = atan2f(imu->accel_y_ms2, imu->accel_z_ms2) * RAD_TO_DEG;
    float accel_roll = atan2f(imu->accel_x_ms2, imu->accel_z_ms2) * RAD_TO_DEG * -1.0f; // Invert roll angle to match right-hand rule

    // Complementary filter: combine gyro and accel angles
    cf->pitch = cf->alpha * pitch_gyro + (1.0f - cf->alpha) * accel_pitch;
    cf->roll = cf->alpha * roll_gyro + (1.0f - cf->alpha) * accel_roll;
}
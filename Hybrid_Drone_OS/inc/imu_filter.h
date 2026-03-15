#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.29577951308232f

typedef struct {
    float pitch;    // Filtered pitch angle in degrees
    float roll;     // Filtered roll angle in degrees
    float alpha;    // Filter coefficient for complementary filter
    uint32_t last_time_ms; // Timestamp of the last update in milliseconds
} CF_State_t;

void CF_Init(CF_State_t *cf, float alpha);
void CF_Update(CF_State_t *cf, MPU6050_Data_t *imu, uint32_t current_time_ms);

#endif /* IMU_FILTER_H */
/*
    * Hybrid_Drone_OS - Main application file
    Author: Joshua (Samir Zambrano)
    Project: Hybrid_Drone_OS
    Pins:
        - LED: PC13
        - UART1: PA9 (TX), PA10 (RX)
        - I2C1: PB6 (SCL), PB7 (SDA)
        - PWM Outputs for motors: PA0 (CH1), PA1 (CH2), PA2 (CH3), PA3 (CH4)
    Peripherals used:
        - SysTick for system timing
        - UART1 for communication with ground station
        - I2C1 for communication with MPU6050 sensor
        - TIM2 for generating PWM signals for motor control
*/
#include "stm32f4xx.h"
#include <stdbool.h>
#include "mpu6050.h"
#include "clock.h"
#include "uart.h"
#include "i2c.h"
#include "pwm.h"
#include "imu_filter.h"
#include "pid.h"

// Global variables
CF_State_t filter; // Complementary filter state
extern volatile uint16_t pwm_duty_cycle_cmd; // Variable to hold the commanded duty cycle from UART input
extern volatile bool cmd_ready; // Flag to indicate a new command is ready to be processed

// ===== Main =====
int main(void) {
    // Configure main system components
    SystemClock_Config(); // Configure the system clock
    SysTick_Config(SystemCoreClock / 1000); // Configure SysTick for 1ms interrupts
    FPU_Enable(); // Enable the FPU before any float operations

    // Initialize peripherals
    UART1_Init(); // Initialize UART1
    I2C1_Init(); // Initialize I2C1
    PWM_Init(); // Initialize PWM for motor control

    // Initialize the complementary filter
    CF_Init(&filter, 0.98f); // Initialize with a filter coefficient of 0.98

    UART1_SendString("===== System initialized =====\r\n"); // Send an initial message
    UART1_SendString("System Clock: 100 MHz\r\n");
    UART1_SendString("SysTick configured for 1 ms ticks\r\n");
    UART1_SendString("FPU enabled\r\n");
    
    // Try to initialize the MPU6050 sensor, retrying if it fails (e.g., due to I2C bus issues)
    UART1_SendString("Initializing MPU6050...\r\n");
    while (MPU6050_Init() == MPU6050_ERR)
    {
        I2C1_Init(); // Re-initialize I2C1 in case of bus issues        
        UART1_SendString("MPU6050 initialization failed\r\n");
        delay_ms(1000); // Wait before retrying
    }
    UART1_SendString("MPU6050 initialized\r\n");

    // ===== Initial variables =====
    uint16_t pwm_duty_cycle = 0; // Initial PWM duty cycle (0%)
    uint32_t last_telemetry_time = system_ticks; // Timestamp for the last telemetry update

    // Set initial PWM duty cycle for all channels
    UART1_SendString("Setting initial PWM duty cycle to 0%\r\n");
    for (uint8_t ch = 1; ch <= 4; ch++) {
        PWM_SetDutyCycle(ch, pwm_duty_cycle);
    }

    UART1_SendString("Entering main loop. Send digits followed by Enter to set CH1-4 duty cycle (0-5000 for 0-100%)\r\n");

    // Main loop
    while (1) {
        // UART command handling (non-blocking)
        if (cmd_ready) {
            // Echo the received command back to the user
            UART1_SendString("\r\n[SYS] Setting CH1-4 duty cycle to ");
            UART1_SendInt(pwm_duty_cycle_cmd);
            UART1_SendString("\r\n");

            for (uint8_t ch = 1; ch <= 4; ch++) {
                PWM_SetDutyCycle(ch, pwm_duty_cycle_cmd);
            }
            
            pwm_duty_cycle_cmd = 0;
            cmd_ready = false;
        }

        // Periodic telemetry update every 10 ms
        if ((system_ticks - last_telemetry_time) >= 10) {
            last_telemetry_time = system_ticks; // Update the timestamp for the last telemetry update
            MPU6050_Data_t sensor_data;
            if (MPU6050_Read(&sensor_data) == MPU6050_OK) {
                MPU6050_Scale(&sensor_data); // Scale raw data to physical units
                CF_Update(&filter, &sensor_data, system_ticks); // Update the complementary filter with new sensor data
                // Send scaled data over UART (for demonstration)
                UART1_SendString("T:");
                UART1_SendInt(system_ticks); // Send the current system tick count as a timestamp
                UART1_SendString(",Ax:");
                UART1_SendFloat(sensor_data.accel_x_ms2, 2);
                UART1_SendString(",Ay:");
                UART1_SendFloat(sensor_data.accel_y_ms2, 2);
                UART1_SendString(",Az:");
                UART1_SendFloat(sensor_data.accel_z_ms2, 2);
                UART1_SendString(",Gx:");
                UART1_SendFloat(sensor_data.gyro_x_dps, 2);
                UART1_SendString(",Gy:");
                UART1_SendFloat(sensor_data.gyro_y_dps, 2);
                UART1_SendString(",Gz:");
                UART1_SendFloat(sensor_data.gyro_z_dps, 2);
                UART1_SendString(",P:");
                UART1_SendFloat(filter.pitch, 2);
                UART1_SendString(",R:");
                UART1_SendFloat(filter.roll, 2);
                UART1_SendString("\r\n");
            } else {
                UART1_SendString("Failed to read from MPU6050\r\n");
            }
        }
    }
}
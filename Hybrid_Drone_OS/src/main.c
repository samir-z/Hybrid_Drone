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
    uint16_t pwm_duty_cycle = 500; // Initial PWM duty cycle (10%)
    uint16_t pwm_duty_cycle_cmd = 0; // Variable to hold the commanded duty cycle from UART input
    uint32_t last_telemetry_time = system_ticks; // Timestamp for the last telemetry update

    // Set initial PWM duty cycle for all channels
    UART1_SendString("Setting initial PWM duty cycle to 10%\r\n");
    for (uint8_t ch = 1; ch <= 4; ch++) {
        PWM_SetDutyCycle(ch, pwm_duty_cycle);
    }

    UART1_SendString("Entering main loop. Send digits followed by Enter to set CH1 duty cycle (0-1000 for 0-100%)\r\n");

    // Main loop
    while (1) {
        // UART command handling (non-blocking)
        if (USART1->SR & USART_SR_RXNE) { // Check if data is available to read
            char cmd = USART1->DR; // Read the received character
            UART1_SendChar(cmd); // Echo the received character back for confirmation

            if (cmd >= '0' && cmd <= '9') {
                pwm_duty_cycle_cmd = (pwm_duty_cycle_cmd * 10) + (cmd - '0'); // Build the duty cycle command from received digits
            }
            else if (cmd == '\r' || cmd == '\n') {  // If the command is complete (Enter key), process it
                UART1_SendString("\r\n[SYS] Setting CH1 duty cycle to "); // Move to the next line after receiving the command
                UART1_SendInt(pwm_duty_cycle_cmd);
                UART1_SendString("\r\n");

                PWM_SetDutyCycle(1, pwm_duty_cycle_cmd); // Set the duty cycle for channel 1 based on the received command
                pwm_duty_cycle_cmd = 0; // Reset the command variable for the next input
            }
        }

        // Periodic telemetry update every 10 ms
        if ((system_ticks - last_telemetry_time) >= 10) {
            last_telemetry_time = system_ticks; // Update the timestamp for the last telemetry update
            MPU6050_Data_t sensor_data;
            if (MPU6050_Read(&sensor_data) == MPU6050_OK) {
                MPU6050_Scale(&sensor_data); // Scale raw data to physical units
                // Send scaled data over UART (for demonstration)
                UART1_SendString("T:");
                UART1_SendInt(system_ticks); // Send the current system tick count as a timestamp
                UART1_SendString(",AX:");
                UART1_SendFloat(sensor_data.accel_x_ms2, 2);
                UART1_SendString(",AY:");
                UART1_SendFloat(sensor_data.accel_y_ms2, 2);
                UART1_SendString(",AZ:");
                UART1_SendFloat(sensor_data.accel_z_ms2, 2);
                UART1_SendString(",GX:");
                UART1_SendFloat(sensor_data.gyro_x_dps, 2);
                UART1_SendString(",GY:");
                UART1_SendFloat(sensor_data.gyro_y_dps, 2);
                UART1_SendString(",GZ:");
                UART1_SendFloat(sensor_data.gyro_z_dps, 2);
                UART1_SendString("\r\n");
            } else {
                UART1_SendString("Failed to read from MPU6050\r\n");
            }
        }
    }
}
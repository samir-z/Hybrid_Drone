/**
 * @file    mixer.c
 * @brief   Implementation of the Mixing Matrix for Quadcopter 'X' configuration.
 */

#include "mixer.h"

// Helper function to keep PWM within physical limits
static float clamp_motor(float value) {
    if (value > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (value < MOTOR_PWM_MIN) return MOTOR_PWM_MIN;
    return value;
}

// Call the appropriate PWM functions to set the duty cycle for each motor (these function is defined in pwm.c)
extern void PWM_SetDutyCycle(uint8_t channel, uint16_t duty); 

void Mixer_Update(float throttle, float pitch_pid, float roll_pid, float yaw_pid, bool is_armed) {
    
    // Kill switch: If the drone is not armed or throttle is zero, set all motors to 0 and return immediately
    if (!is_armed || throttle <= 0.0f) {
        PWM_SetDutyCycle(1, 0);
        PWM_SetDutyCycle(2, 0);
        PWM_SetDutyCycle(3, 0);
        PWM_SetDutyCycle(4, 0);
        return;
    }

    // =====================================================================
    // MIXING MATRIX FOR QUADCOPTER 'X' CONFIGURATION
    // M1: RR(CCW) | M2: FR(CW) | M3: RL(CW) | M4: FL(CCW)
    // =====================================================================
    float m1_cmd = throttle - pitch_pid + roll_pid + yaw_pid; 
    float m2_cmd = throttle + pitch_pid + roll_pid - yaw_pid; 
    float m3_cmd = throttle - pitch_pid - roll_pid - yaw_pid; 
    float m4_cmd = throttle + pitch_pid - roll_pid + yaw_pid; 

    // =====================================================================
    // Clamping
    // =====================================================================
    m1_cmd = clamp_motor(m1_cmd);
    m2_cmd = clamp_motor(m2_cmd);
    m3_cmd = clamp_motor(m3_cmd);
    m4_cmd = clamp_motor(m4_cmd);

    // =====================================================================
    // Set the PWM duty cycle for each motor (convert float to uint16_t as needed by your PWM implementation)
    // Note: Ensure that your PWM_SetDutyCycle function can handle the range of values you are passing (e.g., 0 to 5000 for 0-100% duty cycle
    // =====================================================================
    PWM_SetDutyCycle(1, (uint16_t)m1_cmd);
    PWM_SetDutyCycle(2, (uint16_t)m2_cmd);
    PWM_SetDutyCycle(3, (uint16_t)m3_cmd);
    PWM_SetDutyCycle(4, (uint16_t)m4_cmd);
}
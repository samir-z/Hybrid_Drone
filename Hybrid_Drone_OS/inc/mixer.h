/**
 * @file    mixer.h
 * @brief   Mixing Matrix to Quadcopter 'X' configuration.
 * @author  Joshua (Samir)
 * @note    Map: M1(RR-CCW), M2(FR-CW), M3(RL-CW), M4(FL-CCW).
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include <stdbool.h>

// Define limits for motor PWM signals (adjust according to your hardware configuration)
// (Adjust these values according to the timer resolution on your STM32, e.g., 0 to 5000)
#define MOTOR_PWM_MIN 0.0f
#define MOTOR_PWM_MAX 1000.0f

typedef enum {
    STATE_BOOT,
    STATE_DISARMED, // Safety state: PID inactive, motors locked at 0
    STATE_ARMED     // Danger state: PID active, motors ready to respond to control inputs
} SystemState_t;

/**
 * @brief  Takes the PID effort and distributes it to the 4 motors according to the drone's geometry.
 * @param  throttle    Desired base acceleration (0.0 to MOTOR_PWM_MAX)
 * @param  pitch_pid   PID output for Pitch
 * @param  roll_pid    PID output for Roll
 * @param  yaw_pid     PID output for Yaw (0 if not implemented yet)
 * @param  is_armed    Safety flag. If 'false', the motors are locked at 0.
 */
void Mixer_Update(float throttle, float pitch_pid, float roll_pid, float yaw_pid, bool is_armed);

#endif /* MIXER_H */
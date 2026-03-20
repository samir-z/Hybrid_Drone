/**
 * @file    pid.h
 * @brief   PID controller implementation for the Hybrid Drone OS project.
 * @author  Joshua (Samir Zambrano)
 * @note    Anti-windup implemented using integrator clamping. Derivative term is calculated based on measurement to reduce noise sensitivity.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    // PID Gains
    float Kp;
    float Ki;
    float Kd;
    
    // Filter parameters
    float N;
    float Ts;
    
    // Internal state variables
    float integrator;         // I[n-1]: Integrator accumulator
    float prev_error;         // e[n-1]: Previous error
    float prev_measurement;   // y[n-1]: Previous measurement
    float prev_D_term;        // D[n-1]: Previous derivative term
    
    // Safety limits
    float integrator_limit;   // Maximum limit to avoid Windup
    float output_limit;       // Maximum effort allowed to the motors

    // Constants for the derivative term (pre-calculated for efficiency)
    float A; // Coefficient for the previous D term
    float B; // Coefficient for the measurement difference in the D term
} PID_t;

/**
 * @brief Initializes the PID structure parameters.
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float N, float Ts, float out_limit, float int_limit);

/**
 * @brief Resets the PID memory (useful when arming/disarming the drone).
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Calculates the PID output. Must be called exactly every Ts seconds.
 */
float PID_Update(PID_t *pid, float setpoint, float measurement);

#endif /* PID_H */
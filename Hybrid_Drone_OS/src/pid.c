/**
 * @file    pid.c
 * @brief   Implementation of the PID controller for the Hybrid Drone OS project.
 */

#include "pid.h"

// ============================================================================
// INIT AND RESET
// ============================================================================

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float N, float Ts, float out_limit, float int_limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->N  = N;
    pid->Ts = Ts;

    pid->output_limit     = out_limit;
    pid->integrator_limit = int_limit;

    float denominator = 2.0f + (pid->N * pid->Ts);
    float A = (2.0f - (pid->N * pid->Ts)) / denominator;
    float B = (2.0f * pid->Kd * pid->N) / denominator;

    pid->A = A;
    pid->B = B;
    
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid) {
    pid->integrator       = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_D_term      = 0.0f;
}

// ============================================================================
// CONTROL UPDATE
// ============================================================================

float PID_Update(PID_t *pid, float setpoint, float measurement) {
    
    // Actual error calculation
    float error = setpoint - measurement;

    // ------------------------------------------------------------------------
    // Proportional term (P)
    // ------------------------------------------------------------------------
    float P_term = pid->Kp * error;

    // ------------------------------------------------------------------------
    // Integral term (I) - Tustin's method
    // ------------------------------------------------------------------------
    // I[n] = I[n-1] + (Ki * Ts / 2) * (e[n] + e[n-1])
    float I_term = pid->integrator + 0.5f * pid->Ki * pid->Ts * (error + pid->prev_error);
    
    // Anti-Windup (Clamping)
    // For safety, we clamp the integrator to prevent excessive accumulation (windup) which can lead to instability.
    if (I_term > pid->integrator_limit) {
        I_term = pid->integrator_limit;
    } else if (I_term < -pid->integrator_limit) {
        I_term = -pid->integrator_limit;
    }
    pid->integrator = I_term; // Save the clamped integrator value for the next cycle

    // ------------------------------------------------------------------------
    // Derivative term (D) - Tustin's low-pass filter + Derivative Kick
    // ------------------------------------------------------------------------
    // Derivative Kick. We differentiate the measurement, not the error.
    // D[n] = A * D[n-1] - B * (y[n] - y[n-1])
    
    float D_term = (pid->A * pid->prev_D_term) - (pid->B * (measurement - pid->prev_measurement));
    pid->prev_D_term = D_term; // Save the D term for the next cycle

    // ------------------------------------------------------------------------
    // Total PID output before clamping
    // ------------------------------------------------------------------------
    float output = P_term + I_term + D_term;

    // Clamping the output to the specified limits to ensure we don't command more than the motors can handle.
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // ------------------------------------------------------------------------
    // Update internal state for the next cycle
    // ------------------------------------------------------------------------
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}
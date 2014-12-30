/**
 * @file control_loops_common.h
 * @brief Common declarations for control loops
 */

#pragma once

/**
 * @brief struct definition for PID Controller
 */
typedef struct {
    int Kp_n, Kp_d; //Kp = Kp_n/Kp_d
    int Ki_n, Ki_d; //Ki = Ki_n/Ki_d
    int Kd_n, Kd_d; //Kd = Kd_n/Kd_d
    int Integral_limit;
    int Control_limit;
    int Loop_time;
} ctrl_par;

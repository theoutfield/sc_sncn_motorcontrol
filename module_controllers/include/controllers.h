/**
 * @file controllers.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motion_control_service.h>

/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double integral_limit;
    double integral;
    double actual_value_1n;
    int T_s;    //Sampling-Time in microseconds
} PIDparam;

/**
 * @brief Structure type to set the parameters of nonlinear position controller.
 */
typedef struct {
    double kp;
    double ki;
    double kd;
    double integral_limit_pos;
    double pid_gain;

    double k_fb;         // position feedback gain
    double resolution;   // position sensor resolution
    double gained_error; //position error which is directly measured
    double constant_gain;
    double k_m;          // actuator torque gain

    double feedback_p_loop;
    double feedback_d_loop;

    double y_k;
    double abs_y_k;
    double y_k_sign;
    double y_k_1;
    double delta_y_k;

    double state_1;
    double state_2;
    double state_3;
    double state_min;
    double state_index;

    double dynamic_max_speed; //the maximum speed which the system should have (in order to stop at target with no overshoot)

    double ts_position; // sampling time for position controller [sec]

    double w_max; // maximum speed [rad/sec]
    double t_max; // maximum motor torque [milli-Nm]
    double t_additive; // additive torque [milli-Nm]
    double moment_of_inertia; //moment of inertia
    double calculated_j;

    double torque_ref_k; // milli-Nm

} NonlinearPositionControl;

/**
 * @brief Structure type to set the parameters of position reference profiler.
 */
typedef struct {
    double delta_T;
    double resolution;              //resolution    [ticks/mechanical_rotation]
    double v_max;                   //velocity      [rpm]
    double a_max;                   //acceleration  [rpm/s]
    double torque_rate_max;         //torque rate   [mNm/s]
} posProfilerParam;

/**
 * @brief intializing the parameters of the PID controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_init(PIDparam &param);

/**
 * @brief setting the parameters of the PID controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param structure including the parameters of the PID controller
 *
 * @return void
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDparam &param);

/**
 * @brief updating the PID controller.
 * @param desired_value, the reference set point
 * @param actual_value, the actual value (measurement)
 * @param T_s, sampling time
 * @param param, the structure containing the pid controller parameters
 *
 *
 * @return the output of pid controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDparam &param);

/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDparam &param);

/**
 * @brief resetting the parameters of the nonlinear position controller
 * @param the parameters of the controller
 *
 * @return void
 */
void nl_position_control_reset(NonlinearPositionControl &nl_pos_ctrl);

/**
 * @brief resetting the parameters of nonlinear position controller.
 *
 * @param nl_pos_ctrl, structure containing the parameters of the controller
 * @param pos_velocity_ctrl_config, structure containing the parameters of non-linear position controller
 * @param control_loop_period in us
 *
 * @return void
 */
void nl_position_control_set_parameters(
        NonlinearPositionControl &nl_pos_ctrl,
        MotionControlConfig &pos_velocity_ctrl_config,
        int control_loop_period);

/**
 * @brief updating the output of position controller.
 *
 * @param nl_pos_ctrl, structure containing the parameters of position controller
 * @param position_ref_k_, the reference value of position
 * @param position_sens_k_1_, actual position value in previous sampling
 * @param position_sens_k_, actual position value in current sampling
 *
 * @return the reference value of required torque (in milli-Nm)
 */
int update_nl_position_control(
        NonlinearPositionControl &nl_pos_ctrl,
        double position_ref_k_,
        double position_sens_k_1_,
        double position_sens_k_);

/**
 * @brief updating the velocity reference profiler
 *
 * @param   velocity_ref, target velocity
 * @param   velocity_ref_in_k_1n, profiled velocity calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   velocity_control_loop, the execution cycle of velocity controller (us)
 *
 * @return  profiled velocity calculated for the next step
 */
double velocity_profiler(double velocity_ref, double velocity_ref_in_k_1n, posProfilerParam profiler_param, int velocity_control_loop);

/**
 * @brief updating the torque reference profiler
 *
 * @param   torque_ref, target torque
 * @param   torque_ref_in_k_1n, profiled torque calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   torque_control_loop, the execution cycle of torque controller (us)
 *
 * @return  profiled torque calculated for the next step
 */
double torque_profiler(double torque_ref, double torque_ref_in_k_1n, posProfilerParam profiler_param, int torque_control_loop);

/**
 * @brief updating the position reference profiler
 * @param   pos_target, target position
 * @param   pos_k_1n, profiled position calculated in one step ago
 * @param   pos_k_2n, profiled position calculated in two steps ago
 * @param   pos_profiler_param parameters of the position reference profiler
 *
 * @return  profiled position calculated for the next step
 */
float pos_profiler(double pos_target, double pos_k_1n, double pos_k_2n, posProfilerParam pos_profiler_param);





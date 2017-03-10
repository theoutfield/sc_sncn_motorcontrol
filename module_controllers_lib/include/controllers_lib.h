/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <control_loops_common.h>

#pragma once


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
 * @brief Structure type to set the parameters of nonlinear position controller with saturation.
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
    double j; //moment of inertia
    double calculated_j;

    double torque_ref_k; // milli-Nm

} NonlinearPositionControl;



/**
 * @brief Structure type to set the parameters of the integral optimum position controller.
 */
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double integral_limit;
    double integral;
    double actual_value_1n;
    int T_s;    //Sampling-Time in microseconds
} integralOptimumPosControllerParam;


/**
 * @brief Structure type to set the parameters of position reference profiler.
 */
typedef struct {
    double delta_T;
    double a_max;
    double v_max;
} posProfilerParam;




/**
 * @brief intializing the parameters of the PID controller.
 * @param the parameters of the controller
 */
void pid_init(PIDparam &param);


/**
 * @brief setting the parameters of the PID controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param the parameters of the PID controller
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDparam &param);


/**
 * @brief updating the PID controller.
 * @param output, control command
 * @param input, setpoint
 * @param input, feedback
 * @param input, sample-time in us (microseconds).
 * @param the parameters of the PID controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDparam &param);


/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 */
void pid_reset(PIDparam &param);


/**
 * @brief resetting the parameters of the nonlinear position controller with saturation.
 * @param the parameters of the controller
 */
void nl_position_control_reset(NonlinearPositionControl &nl_pos_ctrl);


/**
 * @brief resetting the parameters of nonlinear position controller with saturation.
 * @param the parameters of the controller
 * @param control_loop_period in us
 */
void nl_position_control_set_parameters(
        NonlinearPositionControl &nl_pos_ctrl,
        PosVelocityControlConfig &pos_velocity_ctrl_config,
        int control_loop_period);


/**
 * @brief updating the output of position controller with update.
 * @param output, torque reference in milli-Nm
 * @param input, setpoint
 * @param input, feedback
 */
int update_nl_position_control(
        NonlinearPositionControl &nl_pos_ctrl,
        double position_ref_k_,
        double position_sens_k_1_,
        double position_sens_k_);


/**
 * @brief updating the position reference profiler
 * @param output, profiled position calculated for the next step
 * @param input, target position
 * @param input, profiled position calculated in one step ago
 * @param input, profiled position calculated in two steps ago
 * @param the parameters of the position reference profiler
 */
float pos_profiler(double pos_target, double pos_k_1n, double pos_k_2n, posProfilerParam pos_profiler_param);








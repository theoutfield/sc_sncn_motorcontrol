/**
 * @file limited_torque_position_control.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**
 * @brief Structure type to set the parameters of limited torque position controller.
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

    double gained_feedback;
    double gained_feedback_k_1;
    double inertia_damping;
    double feedback_d_loop;

    double w_k;
    double abs_w_k;
    double w_k_sign;
    double w_k_1;
    double delta_w_k;

    double max_state_1;
    double max_state_2;
    double max_state_3;
    double max_state_min;
    double max_state_index;

    double dynamic_max_speed; //the maximum speed which the system should have (in order to stop at target with no overshoot)

    double ts_position; // sampling time for position controller [sec]

    double w_max; // maximum speed [rad/sec]
    double t_max; // maximum motor torque [milli-Nm]
    double t_additive; // additive torque [milli-Nm]
    double moment_of_inertia; //moment of inertia
    double calculated_j;

    double torque_ref_k; // milli-Nm

} LimitedTorquePosCtrl;


/**
 * @brief resetting the parameters of the limited torque position controller
 * @param the parameters of the controller
 *
 * @return void
 */
void lt_position_control_reset(LimitedTorquePosCtrl &lt_pos_ctrl);

/**
 * @brief resetting the parameters of limited torque position controller.
 *
 * @param lt_pos_ctrl, structure containing the parameters of the controller
 * @param pos_velocity_ctrl_config, structure containing the parameters of limited torque position controller
 * @param control_loop_period in us
 *
 * @return void
 */
void lt_position_control_set_parameters(LimitedTorquePosCtrl &lt_pos_ctrl,
        int max_motor_speed, int resolution, int moment_of_inertia,
        int position_kp, int position_ki, int position_kd, int position_integral_limit,
        int max_torque, int control_loop_period);

/**
 * @brief updating the output of position controller.
 *
 * @param lt_pos_ctrl, structure containing the parameters of position controller
 * @param position_ref_k_, the reference value of position
 * @param position_sens_k_1_, actual position value in previous sampling
 * @param position_sens_k_, actual position value in current sampling
 *
 * @return the reference value of required torque (in milli-Nm)
 */
int update_lt_position_control(
        LimitedTorquePosCtrl &lt_pos_ctrl,
        double position_ref_k_,
        double position_sens_k_1_,
        double position_sens_k_);




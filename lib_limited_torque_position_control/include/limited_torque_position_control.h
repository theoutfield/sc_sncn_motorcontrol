/**
 * @file limited_torque_position_control.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

// limited torque position controller autotuner
#define AUTO_TUNE_STEP_AMPLITUDE    20000 // The tuning procedure uses steps to evaluate the response of controller. This input is equal to half of step command amplitude.
#define AUTO_TUNE_COUNTER_MAX       1500  // The period of step commands in ticks. Each tick is corresponding to one execution sycle of motion_control_service. As a result, 3000 ticks when the frequency of motion_control_service is 1 ms leads to a period equal to 3 seconds for each step command.
#define PER_THOUSAND_OVERSHOOT      10    // Overshoot limit while tuning (it is set as per thousand of step amplitude)
#define RISE_TIME_FREEDOM_PERCENT   300   // This value helps the tuner to find out whether the ki is high enough or not. By default set this value to 300, and if the tuner is not able to find proper values (and the response is having oscillations), increase this value to 400 or 500.

/**
 * @brief Type for position sensors.
 */
typedef enum {
    FORCE_LOAD_TO_FOLLOW_REFERENCE=1,
    SOFT_LIMIT_OVERSHOOT_AND_ERR_SS=2,
    SHARPEN_CONTROLLER_I=3,
    HARD_LIMIT_OVERSHOOT=4,
    SHARPEN_CONTROLLER_II=5,
    END=6
} AutotuneSteps;


/**
 * @brief Structure type containing auto_tuning parameters of limited torque position controller
 */
typedef struct {

    double position_init;
    double position_ref;
    int rising_edge;

    int activate;
    int counter;
    int counter_max;

    int active_step_counter;
    int active_step;

    int kp;
    int ki;
    int kd;
    int kl;
    int j;
    int auto_tune;

    double step_amplitude;

    double err;
    double err_energy;
    double err_energy_int;
    double err_energy_int_max;
    double dynamic_err_energy_int_max;

    double err_ss;
    double err_energy_ss;
    double err_energy_ss_int;
    double err_energy_ss_int_min;
    int    err_energy_ss_int_min_counter;
    double err_energy_ss_limit_soft;

    double overshoot;
    double overshoot_max;
    double overshoot_min;
    double overshot_per_thousand;
    int overshoot_counter;
    int overshoot_1st_round_damped;

    int rise_time;
    int rise_time_opt;
    int rise_time_freedom_percent;

    int tuning_process_ended;

} LTPosCtrlAutoTuneParam;


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


/**
 * @brief function to initialize the structure of automatic tuner for limited torque position controller.
 *
 * @param lt_pos_ctrl_auto_tune         structure of type LTPosCtrlAutoTuneParam which will be used during the tuning procedure
 * @param step_amplitude Communication  The tuning procedure uses steps to evaluate the response of controller. This input is equal to half of step command amplitude.
 * @param counter_max                   The period of step commands in ticks. Each tick is corresponding to one execution sycle of motion_control_service. As a result, 3000 ticks when the frequency of motion_control_service is 1 ms leads to a period equal to 3 seconds for each step command.
 * @param overshot_per_thousand         Overshoot limit while tuning (it is set as per thousand of step amplitude)
 * @param rise_time_freedom_percent     This value helps the tuner to find out whether the ki is high enough or not. By default set this value to 300, and if the tuner is not able to find proper values (and the response is having oscillations), increase this value to 400 or 500.
 *
 * @return void
 *  */
int init_lt_pos_ctrl_autotune(LTPosCtrlAutoTuneParam &lt_pos_ctrl_auto_tune, int counter_max_autotune, int step_amplitude_autotune, int per_thousand_overshoot_autotune, int rise_time_freedom_percent_autotune);

/**
 * @brief function to automatically tune the limited torque position controller.
 *
 * @param motion_ctrl_config            Structure containing all parameters of motion control service
 * @param lt_pos_ctrl_auto_tune         structure of type LTPosCtrlAutoTuneParam which will be used during the tuning procedure
 * @param position_k                    The actual position of the system while (double)
 *
 * @return void
 *  */
int lt_pos_ctrl_autotune(LTPosCtrlAutoTuneParam &lt_pos_ctrl_auto_tune, double position_k);




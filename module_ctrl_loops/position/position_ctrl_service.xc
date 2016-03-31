/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <xs1.h>
#include <xscope.h>
#include <print.h>
#include <stdlib.h>

#include <position_ctrl_service.h>
#include <a4935.h>
#include <mc_internal_constants.h>
#include <hall_service.h>
#include <qei_service.h>

#include "profile_position_internal.h"

void init_position_control(interface PositionControlInterface client i_position_control)
{
    int ctrl_state;

    while (1) {
        ctrl_state = i_position_control.check_busy();
        if (ctrl_state == INIT_BUSY) {
            i_position_control.enable_position_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("position_ctrl_service: position control initialized");
#endif
            break;
        }
    }
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
    if (position > max_position_limit) {
        position = max_position_limit;
    } else if (position < min_position_limit) {
        position = min_position_limit;
    }
    return position;
}

void position_control_service(ProfilerConfigInternal profiler_config,
                              ControlConfig &position_control_config,
                              interface HallInterface client ?i_hall,
                              interface QEIInterface client ?i_qei,
                              interface BISSInterface client ?i_biss,
                              interface AMSInterface client ?i_ams,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionControlInterface server i_position_control[3])
{
    int actual_position = 0;
    int target_position = 0;
    int error_position = 0;
    int error_position_D = 0;
    int error_position_I = 0;
    int previous_error = 0;
    int position_control_out = 0;

    int position_control_out_limit = 0;
    int error_position_I_limit = 0;

    timer t;
    unsigned int ts;

    int mode = MOTCTRL_MODE_STOP;

    // Profile
    int profile_steps = -1;
    int profile_step_counter = 1;
    timer t_profile;
    uint32_t time_profile;
    ProfilePositionParams profile_pos_params;

    if (!isnull(i_hall)) {
        profile_pos_params.hall_config = i_hall.get_hall_config();
    }

    if (!isnull(i_qei)) {
        profile_pos_params.qei_params = i_qei.get_qei_config();
    }

    if (!isnull(i_biss)) {
        profile_pos_params.biss_config = i_biss.get_biss_config();
    }

    if (!isnull(i_ams)) {
        profile_pos_params.ams_config = i_ams.get_ams_config();
    }

    profile_pos_params.max_position =  profiler_config.max_position;
    profile_pos_params.min_position = profiler_config.min_position;
    profile_pos_params.sensor_used = position_control_config.feedback_sensor;
    if (profile_pos_params.sensor_used == HALL_SENSOR) {
        profile_pos_params.max_acceleration =  position_internal_rpm_to_ticks_hall(profiler_config.max_acceleration, profile_pos_params.hall_config);
        profile_pos_params.max_velocity = position_internal_rpm_to_ticks_hall(profiler_config.max_velocity, profile_pos_params.hall_config);
    } else if (profile_pos_params.sensor_used == QEI_SENSOR) {
        profile_pos_params.max_acceleration =  position_internal_rpm_to_ticks_qei(profiler_config.max_acceleration, profile_pos_params.qei_params);
        profile_pos_params.max_velocity = position_internal_rpm_to_ticks_qei(profiler_config.max_velocity, profile_pos_params.qei_params);
    } else if (profile_pos_params.sensor_used == BISS_SENSOR) {
        profile_pos_params.max_acceleration =  position_internal_rpm_to_ticks_biss(profiler_config.max_acceleration, profile_pos_params.biss_config);
        profile_pos_params.max_velocity = position_internal_rpm_to_ticks_biss(profiler_config.max_velocity, profile_pos_params.biss_config);
    } else if (profile_pos_params.sensor_used == AMS_SENSOR) {
        profile_pos_params.max_acceleration =  position_internal_rpm_to_ticks_ams(profiler_config.max_acceleration, profile_pos_params.ams_config);
        profile_pos_params.max_velocity = position_internal_rpm_to_ticks_ams(profiler_config.max_velocity, profile_pos_params.ams_config);
    }
    profile_pos_params.limit_factor = 10;

    // Notification
    int notification = MOTCTRL_NTF_EMPTY;

    int config_update_flag = 1;
    MotorcontrolConfig motorcontrol_config;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;

    while(1) {
#pragma ordered
        select {
            case (profile_step_counter <= profile_steps) => t_profile when timerafter(time_profile + MSEC_STD) :> time_profile:
                if (profile_step_counter == profile_steps) {
                    profile_steps = -1;
                    profile_step_counter = 1;

                    notification = MOTCTRL_NTF_PROFILE_DONE;
                    // TODO: Use a constant for the number of interfaces
                    for (int i = 0; i < 3; i++) {
                        i_position_control[i].notification();
                    }
                } else {
                    target_position = generate_profile_step_position(profile_pos_params, profile_step_counter);
                    profile_step_counter++;
                }
                break;
            case t when timerafter(ts + USEC_STD * position_control_config.control_loop_period) :> ts:
                if (config_update_flag) {
                    motorcontrol_config = i_motorcontrol.get_config();

                    //Limits
                    if (motorcontrol_config.motor_type == BLDC_MOTOR) {
                        if(motorcontrol_config.commutation_method == FOC){
                            position_control_out_limit = 4096;//FOC control range [-4096:4096]
                        } else {
                            position_control_out_limit = BLDC_PWM_CONTROL_LIMIT;
                        }
                    } else if(motorcontrol_config.motor_type == BDC_MOTOR) {
                        position_control_out_limit = BDC_PWM_CONTROL_LIMIT;
                    }

                    if (position_control_config.feedback_sensor != HALL_SENSOR
                           && position_control_config.feedback_sensor != QEI_SENSOR
                           && position_control_config.feedback_sensor != BISS_SENSOR
                           && position_control_config.feedback_sensor != AMS_SENSOR) {
                        position_control_config.feedback_sensor = motorcontrol_config.commutation_sensor;
                    }

                    if (position_control_config.Ki_n != 0) {
                        error_position_I_limit = position_control_out_limit * PID_DENOMINATOR / position_control_config.Ki_n;
                    }

                    if (position_control_config.feedback_sensor == HALL_SENSOR && !isnull(i_hall)) {
                        actual_position = i_hall.get_hall_position_absolute();
                    } else if (position_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
                        actual_position = i_qei.get_qei_position_absolute();
                    } else if (position_control_config.feedback_sensor == BISS_SENSOR && !isnull(i_biss)) {
                        { actual_position, void, void } = i_biss.get_biss_position();
                    } else if (position_control_config.feedback_sensor == AMS_SENSOR && !isnull(i_ams)) {
                        { actual_position, void } = i_ams.get_ams_position();
                    }

                    config_update_flag = 0;
                }

                /* acquire actual position */
                switch (position_control_config.feedback_sensor) {
                    case HALL_SENSOR:
                        if (!isnull(i_hall)) {
                            actual_position = i_hall.get_hall_position_absolute();
                        } else {
                            printstrln("position_ctrl_service: ERROR: Hall interface is not provided but requested");
                            exit(-1);
                        }
                        break;

                    case QEI_SENSOR:
                        if (!isnull(i_qei)) {
                            actual_position =  i_qei.get_qei_position_absolute();
                        } else {
                            printstrln("position_ctrl_service: ERROR: Encoder interface is not provided but requested");
                            exit(-1);
                        }
                        break;

                    case BISS_SENSOR:
                        if (!isnull(i_biss)) {
                            { actual_position, void, void } = i_biss.get_biss_position();
                        } else {
                            printstrln("position_ctrl_service: ERROR: BiSS interface is not provided but requested");
                            exit(-1);
                        }
                        break;

                    case AMS_SENSOR:
                        if (!isnull(i_ams)) {
                            { actual_position, void } = i_ams.get_ams_position();
                        } else {
                            printstrln("position_ctrl_service: ERROR: AMS interface is not provided but requested");
                            exit(-1);
                        }
                        break;

                }
                /*
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire position
                 */

                if(mode == MOTCTRL_MODE_ACTIVE) {
                    /* PID Controller */

                    error_position = (target_position - actual_position);
                    error_position_I = error_position_I + error_position;
                    error_position_D = error_position - previous_error;

                    if (error_position_I > error_position_I_limit) {
                        error_position_I = error_position_I_limit;
                    } else if (error_position_I < -error_position_I_limit) {
                        error_position_I = - error_position_I_limit;
                    }

                    position_control_out = (position_control_config.Kp_n * error_position) +
                                           (position_control_config.Ki_n * error_position_I) +
                                           (position_control_config.Kd_n * error_position_D);

                    position_control_out /= PID_DENOMINATOR;

                    if (position_control_out > position_control_out_limit) {
                        position_control_out = position_control_out_limit;
                    } else if (position_control_out < -position_control_out_limit) {
                        position_control_out =  -position_control_out_limit;
                    }


                    if(position_control_config.cascade_with_torque == 1 && motorcontrol_config.commutation_method == FOC){
                        i_motorcontrol.set_torque(position_control_out);  //cascades with FOC's torque controller
                    }
                    else {
                        i_motorcontrol.set_voltage(position_control_out); //in case of FOC sets Q value, torque controller in FOC is disabled
                    }

#ifdef DEBUG
                    xscope_int(ACTUAL_POSITION, actual_position);
                    xscope_int(TARGET_POSITION, target_position);
#endif
                    //xscope_int(TARGET_POSITION, target_position);
                    previous_error = error_position;
                }

                break;

            case !isnull(i_hall) => i_hall.notification():

                switch (i_hall.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case !isnull(i_qei) => i_qei.notification():

                switch (i_qei.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_motorcontrol.notification():

                switch (i_motorcontrol.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_position_control[int i].set_position(int in_target_position):

                if (mode == MOTCTRL_MODE_ACTIVE) {
                    target_position = in_target_position;
                }
                break;

            case i_position_control[int i].get_position() -> int out_position:

                out_position = actual_position;
                break;

            case i_position_control[int i].get_target_position() -> int out_target_position:

                out_target_position = target_position;
                break;

            case i_position_control[int i].set_profile_position(int in_target_position, int velocity, int acceleration, int deceleration):
                profile_pos_params.qf = in_target_position;
                profile_pos_params.qi = actual_position;
                profile_pos_params.vi = velocity;
                profile_pos_params.acc = acceleration;
                profile_pos_params.dec = deceleration;
                profile_steps = calculate_profile_position_steps(profile_pos_params);
                profile_step_counter = 1;
                t_profile :> time_profile;
                break;

            case i_position_control[int i].set_position_control_config(ControlConfig in_params):

                position_control_config = in_params;
                config_update_flag = 1;
                break;

            case i_position_control[int i].get_position_control_config() ->  ControlConfig out_config:

                out_config = position_control_config;
                break;

            case i_position_control[int i].set_position_sensor(int in_sensor_used):

                position_control_config.feedback_sensor = in_sensor_used;
                target_position = actual_position;
                config_update_flag = 1;

                break;

            case i_position_control[int i].check_busy() -> int out_state:

                if (mode < MOTCTRL_MODE_ACTIVE) {
                    out_state = INIT_BUSY;
                } else {
                    out_state = INIT;
                }
                break;

            case i_position_control[int i].enable_position_ctrl():

                mode = MOTCTRL_MODE_ACTIVE;
                if (position_control_config.feedback_sensor == HALL_SENSOR && !isnull(i_hall)) {
                    actual_position = i_hall.get_hall_position_absolute();
                } else if (position_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
                    actual_position = i_qei.get_qei_position_absolute();
                } else if (position_control_config.feedback_sensor == BISS_SENSOR && !isnull(i_biss)) {
                    { actual_position, void, void } = i_biss.get_biss_position();
                } else if (position_control_config.feedback_sensor == AMS_SENSOR && !isnull(i_ams)) {
                    { actual_position, void } = i_ams.get_ams_position();
                }
                target_position = actual_position;

                while (1) {
                    if (i_motorcontrol.check_busy() == INIT) { //__check_commutation_init(c_commutation);
#ifdef debug_print
                        printstrln("position_ctrl_service: commutation initialized");
#endif
                        if (i_motorcontrol.get_fets_state() == 0) { // check_fet_state(c_commutation);
                            i_motorcontrol.set_fets_state(1);
                            delay_milliseconds(2);
                        }

                        break;
                    }
                }
#ifdef debug_print
                printstrln("position_ctrl_service: position control activated");
#endif
                break;

            case i_position_control[int i].disable_position_ctrl():

                mode = MOTCTRL_MODE_STOP;
                i_motorcontrol.set_voltage(0); //set_commutation_sinusoidal(c_commutation, 0);
                error_position = 0;
                error_position_D = 0;
                error_position_I = 0;
                previous_error = 0;
                position_control_out = 0;
                i_motorcontrol.set_fets_state(0); // disable_motor(c_commutation);
                delay_milliseconds(30); //wait_ms(30, 1, ts); //
#ifdef debug_print
                printstrln("position_ctrl_service: position control disabled");
#endif
                break;
            case i_position_control[int i].enable_passive_mode():

                mode = MOTCTRL_MODE_PASSIVE;
                while (1) {
                    if (i_motorcontrol.check_busy() == INIT) { //__check_commutation_init(c_commutation);
                        i_motorcontrol.set_voltage(0);
                        if (i_motorcontrol.get_fets_state() == 1) { //check_fet_state(c_commutation);
                            i_motorcontrol.set_fets_state(0); //enable_motor(c_commutation);
                            delay_milliseconds(2); //wait_ms(2, 1, t);
                        }
                        break;
                    }
                }
                break;
            case i_position_control[int i].get_mode() -> int out_mode:
                out_mode = mode;
                break;
            case i_position_control[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;
        }
    }
}


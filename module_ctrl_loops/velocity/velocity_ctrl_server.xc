/**
 * @file velocity_ctrl_server.xc
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <velocity_ctrl_server.h>
#include <velocity_ctrl_common.h>
#include <refclk.h>
#include <qei_client.h>
#include <commutation_client.h>
#include <filter_blocks.h>
#include <xscope.h>
#include <internal_config.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <stdio.h>
#include <a4935.h>

#if(MOTOR_TYPE == BDC)
#include <brushed_dc_client.h>
#endif

//#define Debug_velocity_ctrl  //don't forget to set up the config.xscope file
//#define debug_print

#define VELOCITY_CTRL_WRITE(x)  c_velocity_ctrl <: (x)
#define VELOCITY_CTRL_READ(x)   c_velocity_ctrl :> (x)

[[combinable]]
void velocity_control( ctrl_par & velocity_ctrl_params,
                       filter_par & sensor_filter_params,
                       hall_par &?hall_params,
                       qei_par &?qei_params,
                       int sensor_used,
                       chanend c_hall,
                       chanend ?c_qei,
                       chanend c_velocity_ctrl,
                       chanend c_commutation )
{
    /* Controller declarations */
    int actual_velocity = 0;
    int target_velocity = 0;
    int error_velocity = 0;
    int error_velocity_D = 0;
    int error_velocity_I = 0;
    int previous_error = 0;
    int velocity_control_out = 0;

    timer ts;
    unsigned int time;

    /* Sensor filter declarations */
    int filter_length = sensor_filter_params.filter_length;
    int filter_buffer[FILTER_SIZE_MAX] = {0};   //default size used at compile time (cant be changed further)
    int index = 0;

    /* speed calc declarations */
    int position;
    int init = 0;
    int previous_position = 0;
    int raw_speed = 0;                      // rpm
    int difference;
    int direction = 0;
    int old_difference;
    int rpm_constant = 1000*60; // constant
    int speed_factor_hall = 0;
    int speed_factor_qei = 0;
    int command;
    int activate = 0;
    int init_state = INIT_BUSY;
    int qei_crossover = 0;
    int hall_crossover = 0;
    int compute_flag = 0;
    int fet_state = 0;

    init_filter(filter_buffer, index, FILTER_SIZE_MAX);
    if (sensor_used == HALL){
        if(velocity_ctrl_params.Loop_time == MSEC_FAST){//FixMe: implement reference clock check
            speed_factor_hall = hall_params.pole_pairs*4096*(velocity_ctrl_params.Loop_time/MSEC_FAST); // variable pole_pairs
        }
        else {
            speed_factor_hall = hall_params.pole_pairs*4096*(velocity_ctrl_params.Loop_time/MSEC_STD); // variable pole_pairs
        }
        hall_crossover = hall_params.max_ticks - hall_params.max_ticks/10;
    }
    else if (sensor_used == QEI){
        if(velocity_ctrl_params.Loop_time == MSEC_FAST){//FixMe: implement reference clock check
            speed_factor_qei = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_FAST);       // variable qei_real_max
        }
        else {
            speed_factor_qei = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);       // variable qei_real_max
        }
        qei_crossover = qei_params.real_counts - qei_params.real_counts/10;
    }

    printf("*************************************\n    VELOCITY CONTROLLER STARTING\n*************************************\n");

    ts :> time;
    time += velocity_ctrl_params.Loop_time;

    init_state = INIT;

    while (1) {
//#pragma ordered
        select {
        case ts when timerafter (time) :> void:
            time += velocity_ctrl_params.Loop_time;
            if (compute_flag == 1) {
                /* calculate actual velocity from hall/qei with filter*/
                if (sensor_used == HALL) {
                    if (init == 0) {
                        { position, direction } = get_hall_position_absolute(c_hall);
                        if (position > 2049) {
                            init = 1;
                            previous_position = 2049;
                        } else if (position < -2049) {
                            init = 1;
                            previous_position = -2049;
                        }
                        raw_speed = 0;
                        //target_velocity = 0;
                    } else if (init == 1) {
                        { position, direction } = get_hall_position_absolute(c_hall);
                        difference = position - previous_position;
                        if (difference > hall_crossover) {
                            difference = old_difference;
                        } else if (difference < -hall_crossover) {
                            difference = old_difference;
                        }
                        raw_speed = (difference*rpm_constant)/speed_factor_hall;
#ifdef Debug_velocity_ctrl
                        //xscope_int(RAW_SPEED, raw_speed);
#endif
                        previous_position = position;
                        old_difference = difference;
                    }
                } else if (sensor_used == QEI && !isnull(c_qei)) {
                    { position, direction } = get_qei_position_absolute(c_qei);
                    difference = position - previous_position;

                    if (difference > qei_crossover) {
                        difference = old_difference;
                    }

                    if (difference < -qei_crossover) {
                        difference = old_difference;
                    }

                    raw_speed = (difference*rpm_constant)/speed_factor_qei;

#ifdef Debug_velocity_ctrl
                    //xscope_int(RAW_SPEED, raw_speed);
#endif
                    previous_position = position;
                    old_difference = difference;
                }
                /**
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire velocity/position
                 */

                actual_velocity = filter(filter_buffer, index, filter_length, raw_speed);
            }

            if(activate == 1) {
#ifdef Debug_velocity_ctrl
                xscope_int(ACTUAL_VELOCITY, actual_velocity);
                xscope_int(TARGET_VELOCITY, target_velocity);
#endif
                compute_flag = 1;
                /* Controller */
                error_velocity   = (target_velocity - actual_velocity);
                error_velocity_I = error_velocity_I + error_velocity;
                error_velocity_D = error_velocity - previous_error;

                if (error_velocity_I > (velocity_ctrl_params.Integral_limit)) {
                    error_velocity_I = (velocity_ctrl_params.Integral_limit);
                } else if (error_velocity_I < -(velocity_ctrl_params.Integral_limit)) {
                    error_velocity_I = 0 -(velocity_ctrl_params.Integral_limit);
                }

                velocity_control_out = ( (velocity_ctrl_params.Kp_n*error_velocity) / (velocity_ctrl_params.Kp_d) +
                                         (velocity_ctrl_params.Ki_n*error_velocity_I) / (velocity_ctrl_params.Ki_d) +
                                         (velocity_ctrl_params.Kd_n*error_velocity_D) / (velocity_ctrl_params.Kd_d) );

                if (velocity_control_out > velocity_ctrl_params.Control_limit) {
                    velocity_control_out = velocity_ctrl_params.Control_limit;
                } else if (velocity_control_out < -velocity_ctrl_params.Control_limit) {
                    velocity_control_out = 0 - velocity_ctrl_params.Control_limit;
                }

#if(MOTOR_TYPE == BDC)
                set_bdc_voltage(c_commutation, velocity_control_out);
#else
                set_commutation_sinusoidal(c_commutation, velocity_control_out);//velocity_control_out
#endif
                previous_error = error_velocity;

            }
    //        printf("looping %d\n", velocity_ctrl_params.Loop_time);
            break;

            /* acq target velocity etherCAT */
        case VELOCITY_CTRL_READ(command):
            switch (command) {
            case VCTRL_CMD_SET_VELOCITY:
                VELOCITY_CTRL_READ(target_velocity);
              //  target_velocity = 1000;
              //  printf("%d %d\n", target_velocity, MOTOR_TYPE);
                break;

            case VCTRL_CMD_GET_VELOCITY:
                VELOCITY_CTRL_WRITE(actual_velocity);
                break;

            case VCTRL_CMD_SET_HALL:
                VELOCITY_CTRL_READ(hall_params.pole_pairs);
                VELOCITY_CTRL_READ(hall_params.max_ticks);
                VELOCITY_CTRL_READ(hall_params.max_ticks_per_turn);

                break;

            case VCTRL_CMD_SET_QEI:
                VELOCITY_CTRL_READ(qei_params.max_ticks);
                VELOCITY_CTRL_READ(qei_params.index);
                VELOCITY_CTRL_READ(qei_params.real_counts);
                VELOCITY_CTRL_READ(qei_params.max_ticks_per_turn);
                VELOCITY_CTRL_READ(qei_params.poles);

                break;

            case SET_VELOCITY_FILTER:
                VELOCITY_CTRL_READ(filter_length);
                if(filter_length > FILTER_SIZE_MAX)
                    filter_length = FILTER_SIZE_MAX;
                break;

            case SET_CTRL_PARAMETER:
                VELOCITY_CTRL_READ(velocity_ctrl_params.Kp_n);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Kp_d);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Ki_n);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Ki_d);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Kd_n);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Kd_d);
                VELOCITY_CTRL_READ(velocity_ctrl_params.Integral_limit);
                break;

            case SENSOR_SELECT:
                VELOCITY_CTRL_READ(sensor_used);
                if(sensor_used == HALL) {
                    speed_factor_hall = hall_params.pole_pairs * 4096 * (velocity_ctrl_params.Loop_time/MSEC_STD);
                    hall_crossover = hall_params.max_ticks - hall_params.max_ticks/10;
                    target_velocity =  actual_velocity;
                } else if(sensor_used == QEI) {
                    speed_factor_qei = qei_params.real_counts * (velocity_ctrl_params.Loop_time/MSEC_STD);
                    qei_crossover = qei_params.max_ticks - qei_params.max_ticks/10;
                    target_velocity = actual_velocity;
                }
                /**
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire velocity/position
                 */
                break;

            case VCTRL_CMD_ENABLE:
                VELOCITY_CTRL_READ(activate);
                activate = SET;
                while (1) {
                    init_state = __check_commutation_init(c_commutation);
                    if (init_state == INIT) {
#ifdef debug_print
                        printf("commutation intialized\n");
#endif
#if(MOTOR_TYPE == BLDC)
                        fet_state = check_fet_state(c_commutation);
                        if (fet_state == 1) {
                            enable_motor(c_commutation);
                            wait_ms(2, 1, ts);
                        }
#endif
                        break;
                    }
                }
#ifdef debug_print
                printf("velocity control activated\n");
#endif
                break;

            case VCTRL_CMD_SHUTDOWN:
                VELOCITY_CTRL_READ(activate);
                error_velocity = 0;
                error_velocity_D = 0;
                error_velocity_I = 0;
                previous_error = 0;
                velocity_control_out = 0;
                set_commutation_sinusoidal(c_commutation, 0);
                disable_motor(c_commutation);
                wait_ms(30, 1, ts);
                break;

            case CHECK_BUSY:
                VELOCITY_CTRL_WRITE(activate);
                break;

            case VCTRL_CMD_GET_STATUS:
                VELOCITY_CTRL_WRITE(activate);
                break;

            default:
                break;
            }
            break;
        }
    }
}

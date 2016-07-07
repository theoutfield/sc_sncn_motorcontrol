/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>


int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
{
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    delay_milliseconds(30000);

    int offset=i_motorcontrol.set_calib(0);
    printf("Detected offset is: %i\n", offset);
//    printf(">>  CHECK PROPER OFFSET POLARITY ...\n");
    int proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();
    if(proper_sensor_polarity == 1) {
        printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
    } else {
        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
    }
    return offset;
}


void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_commutation,
                       interface PositionVelocityCtrlInterface client ?i_position_control)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    DownstreamControlData downstream_control_data;
    PosVelocityControlConfig pos_velocity_ctrl_config;

    int torque = 0;


    pos_velocity_ctrl_config.control_loop_period = CONTROL_LOOP_PERIOD; //us

    pos_velocity_ctrl_config.int21_min_position = MIN_POSITION_LIMIT;
    pos_velocity_ctrl_config.int21_max_position = MAX_POSITION_LIMIT;
    pos_velocity_ctrl_config.int21_max_speed = MAX_VELOCITY;
    pos_velocity_ctrl_config.int21_max_torque = MAX_TORQUE;


    pos_velocity_ctrl_config.int10_P_position = POSITION_Kp;
    pos_velocity_ctrl_config.int10_I_position = POSITION_Ki;
    pos_velocity_ctrl_config.int10_D_position = POSITION_Kd;
    pos_velocity_ctrl_config.int21_P_error_limit_position = POSITION_P_ERROR_lIMIT;
    pos_velocity_ctrl_config.int21_I_error_limit_position = POSITION_I_ERROR_lIMIT;
    pos_velocity_ctrl_config.int22_integral_limit_position = POSITION_INTEGRAL_LIMIT;

    pos_velocity_ctrl_config.int10_P_velocity = VELOCITY_Kp;
    pos_velocity_ctrl_config.int10_I_velocity = VELOCITY_Ki;
    pos_velocity_ctrl_config.int10_D_velocity = VELOCITY_Kd;
    pos_velocity_ctrl_config.int21_P_error_limit_velocity = VELOCITY_P_ERROR_lIMIT;
    pos_velocity_ctrl_config.int21_I_error_limit_velocity = VELOCITY_I_ERROR_lIMIT;
    pos_velocity_ctrl_config.int22_integral_limit_velocity = VELOCITY_INTEGRAL_LIMIT;

    pos_velocity_ctrl_config.position_ref_fc = POSITION_REF_FC;
    pos_velocity_ctrl_config.position_fc = POSITION_FC;
    pos_velocity_ctrl_config.velocity_ref_fc = VELOCITY_REF_FC;
    pos_velocity_ctrl_config.velocity_fc = VELOCITY_FC;
    pos_velocity_ctrl_config.velocity_d_fc = VELOCITY_D_FC;


    delay_milliseconds(2000);
//    i_commutation.set_brake_status(1);

    i_commutation.set_offset_value(1000);
    delay_milliseconds(1000);

    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);

    fflush(stdout);
    //read and adjust the offset.
    while (1) {
        char mode = 0;
        char mode_2 = 0;
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            } else if (c == '-') {
                sign = -1;
            } else if (c != ' ') {
                if (mode == 0) {
                    mode = c;
                } else {
                    mode_2 = c;
                }
            }
        }
        switch(mode) {

        case 'u':
            i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
            printf("control config updated");
            break;
        //velocity pid coefficients
        case 'k':
            switch(mode_2) {
            case 'p':
                pos_velocity_ctrl_config.int10_P_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d\n", pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity);
                break;
            case 'i':
                pos_velocity_ctrl_config.int10_I_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d\n", pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity);
                break;
            case 'd':
                pos_velocity_ctrl_config.int10_D_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d\n", pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity);
                break;
            default:
                printf("Kp:%d Ki:%d Kd:%d\n", pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity);
                break;
            }
            break;
        //velocity pid limits
        case 'l':
            switch(mode_2) {
            case 'p':
                pos_velocity_ctrl_config.int21_P_error_limit_velocity = value * sign;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            case 'i':
                pos_velocity_ctrl_config.int21_I_error_limit_velocity = value * sign;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            case 'l':
                pos_velocity_ctrl_config.int22_integral_limit_velocity = value * sign;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            case 'c':
                pos_velocity_ctrl_config.int21_max_torque = value * sign;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            default:
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            }
            break;

            //position pid coefficients
            case 'p':
                switch(mode_2) {
                case 'p':
                    pos_velocity_ctrl_config.int10_P_position = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("Pp:%d Pi:%d Pd:%d\n", pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position);
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int10_I_position = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("Pp:%d Pi:%d Pd:%d\n", pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position);
                    break;
                case 'd':
                    pos_velocity_ctrl_config.int10_D_position = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("Pp:%d Pi:%d Pd:%d\n", pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position);
                    break;
                default:
                    printf("Pp:%d Pi:%d Pd:%d\n", pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position);
                    break;
                }
                break;
            //position pid limits
            case 'i':
                switch(mode_2) {
                case 'p':
                    pos_velocity_ctrl_config.int21_P_error_limit_position = value * sign;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int21_I_error_limit_position = value * sign;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                case 'l':
                    pos_velocity_ctrl_config.int22_integral_limit_position = value * sign;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                case 'c':
                    pos_velocity_ctrl_config.int21_max_speed = value * sign;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                default:
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                }
                break;

        case 's':
            switch(mode_2) {
                case 'p':
                    if (value == 1) {
                        i_position_control.enable_position_ctrl();
                        printf("position ctrl enabled\n");
                    }
                    else {
                        i_position_control.disable();
                        printf("position ctrl disabled\n");
                    }
                    break;
                case 'v':
                    if (value == 1) {
                        i_position_control.enable_velocity_ctrl();
                        printf("velocity ctrl enabled\n");
                    }
                    else {
                        i_position_control.disable();
                        printf("velocity ctrl disabled\n");
                    }
                    break;
                case 't':
                    if (value == 1) {
                        i_position_control.enable_torque_ctrl();
                        printf("torque ctrl enabled\n");
                    }
                    else {
                        i_position_control.disable();
                        printf("torque ctrl disabled\n");
                    }
                    break;
                }
            break;

        case 'c':
            switch(mode_2) {
                case 'p':
                    printf("position cmd: %d to %d (range:-32767 to 32767)\n", value*sign, -value*sign);
                    for (int ii=0; ii<10; ii++) {
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.position_cmd = value*sign;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(2000);
                        downstream_control_data.position_cmd = -value*sign;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(2000);
                        downstream_control_data.position_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                    }
                    break;
                case 'v':
                    printf("velocity cmd: %d to %d (range:-32767 to 32767)\n", value*sign, -value*sign);
                    downstream_control_data.offset_torque = 0;
                    downstream_control_data.velocity_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(500);
                    downstream_control_data.velocity_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(500);
                    downstream_control_data.velocity_cmd = 0;//value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 't':
                    printf("torque cmd: %d to %d (range:-32767 to 32767)\n", value*sign, -value*sign);
                    downstream_control_data.torque_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(2000);
                    downstream_control_data.torque_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(2000);
                    downstream_control_data.torque_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 'o':
                    printf("offset-torque cmd: %d to %d\n", value*sign, -value*sign);
                    downstream_control_data.position_cmd = 0;
                    downstream_control_data.velocity_cmd = 0;
                    downstream_control_data.offset_torque = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(200);
                    downstream_control_data.offset_torque = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(200);
                    downstream_control_data.offset_torque = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                    }
            break;

        //auto offset tuning
        case 'a':
            auto_offset(i_commutation);
            break;

        //set offset
        case 'o':
            i_commutation.set_offset_value(value);
            printf("set offset to %d\n", value);
            break;

        //reverse torque
        case 'r':
            torque = -torque;
            i_commutation.set_torque(torque);
            printf("Torque %d\n", torque);
            break;

        //reverse torque
        case 'j':
            i_commutation.set_brake_status(1);
            break;

        case 'z':
            i_commutation.reset_faults();
            break;

        //set torque
        default:
            torque = value*sign;
            i_commutation.set_torque(torque);
            printf("Torque %d\n", torque);
            break;
        }
        delay_milliseconds(10);
    }
}

void position_limiter(int position_limit, interface PositionLimiterInterface server i_position_limiter, client interface MotorcontrolInterface i_motorcontrol)
{
    timer t;
    unsigned ts;
    t :> ts;
    int print_position_limit = 0;
    int count = 0;
    int velocity = 0;

    while(1) {
        select {
        case t when timerafter(ts) :> void:

            count = i_motorcontrol.get_position_actual();
            velocity = i_motorcontrol.get_velocity_actual();

            //postion limiter
            if (position_limit > 0) {
                if (count >= position_limit && velocity > 10) {
                    i_motorcontrol.set_torque_control_disabled();
                    i_motorcontrol.set_safe_torque_off_enabled();
                    i_motorcontrol.set_brake_status(0);
                    if (print_position_limit >= 0) {
                        print_position_limit = -1;
                        printf("up limit reached\n");
                    }
                } else if (count <= -position_limit && velocity < -10) {
                    i_motorcontrol.set_torque_control_disabled();
                    i_motorcontrol.set_safe_torque_off_enabled();
                    i_motorcontrol.set_brake_status(0);
                    if (print_position_limit <= 0) {
                        print_position_limit = 1;
                        printf("down limit reached\n");
                    }
                }
            }
            t :> ts;
            ts += USEC_FAST * 1000;
            break;

        case i_position_limiter.set_limit(int in_limit):
            if (in_limit < 0) {
                position_limit = in_limit;
                printf("Position limit disabled\n");
            } else if (in_limit > 0) {
                printf("Position limited to %d ticks\n", in_limit);
                position_limit = in_limit;
            }
            break;

        case i_position_limiter.get_limit() -> int out_limit:
            out_limit =  position_limit;
            break;

        }//end select
    }//end while
}//end function

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
    printf("\n\n\n\n\nsending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    delay_milliseconds(30000);

    int offset=i_motorcontrol.set_calib(0);
    printf("detected offset is: %i\n", offset);
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

    pos_velocity_ctrl_config.int21_min_position = -1000000;
    pos_velocity_ctrl_config.int21_max_position = 1000000;
    pos_velocity_ctrl_config.int21_max_speed = 400;
    pos_velocity_ctrl_config.int21_max_torque = 1200000;


    pos_velocity_ctrl_config.int10_P_position = 40;
    pos_velocity_ctrl_config.int10_I_position = 50;
    pos_velocity_ctrl_config.int10_D_position = 0;
    pos_velocity_ctrl_config.int21_P_error_limit_position = 40000;
    pos_velocity_ctrl_config.int21_I_error_limit_position = 5;
    pos_velocity_ctrl_config.int22_integral_limit_position = 10000;

    pos_velocity_ctrl_config.int10_P_velocity = 60;
    pos_velocity_ctrl_config.int10_I_velocity = 0;
    pos_velocity_ctrl_config.int10_D_velocity = 65;
    pos_velocity_ctrl_config.int21_P_error_limit_velocity = 200000;
    pos_velocity_ctrl_config.int21_I_error_limit_velocity = 0;
    pos_velocity_ctrl_config.int22_integral_limit_velocity = 0;


    delay_milliseconds(2000);
//    i_commutation.set_brake_status(1);

    i_commutation.set_offset_value(1415); //A2
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
                    downstream_control_data.offset_torque = 0;
                    downstream_control_data.position_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.position_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.position_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
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
                    delay_milliseconds(400);
                    downstream_control_data.torque_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(400);
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

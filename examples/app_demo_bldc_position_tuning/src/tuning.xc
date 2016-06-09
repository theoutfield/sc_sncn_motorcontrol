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
        i_motorcontrol.set_torque_control_enabled();
    } else {
        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
    }
    return offset;
}

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol,
                       interface PositionVelocityCtrlInterface client ?i_position_control)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    PosVelocityControlConfig pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();

    DownstreamControlData downstream_control_data;

    int torque = 0;
    int position_ctrl_flag = 0;
    int brake_flag = 0;
    int torque_control_flag = 1;

    i_motorcontrol.set_brake_status(brake_flag);
    i_motorcontrol.set_torque_control_enabled();

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
        //go to position directly
        case 'd':
            downstream_control_data.offset_torque = 0;
            downstream_control_data.position_cmd = value*sign;
            i_position_control.update_control_data(downstream_control_data);
            printf("Go to %d\n", value*sign);
            break;

        //set velocity
        case 'v':
            downstream_control_data.offset_torque = 0;
            downstream_control_data.velocity_cmd = value*sign;
            i_position_control.update_control_data(downstream_control_data);
            printf("set velocity %d\n", value*sign);
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

        //enable
        case 's':
            if (value == 1) {
                position_ctrl_flag = 1;
                switch(mode_2) {
                    case 'p':
                        i_position_control.enable_position_ctrl();
                        printf("position ctrl enabled\n");
                        break;
                    case 'v':
                        i_position_control.enable_velocity_ctrl();
                        printf("velocity ctrl enabled\n");
                        break;
                    case 't':
                        i_position_control.enable_torque_ctrl();
                        printf("torque ctrl enabled\n");
                        break;
                }
            } else {
                position_ctrl_flag = 0;
                torque_control_flag = 0;
                i_position_control.disable();
                printf("position ctrl disabled\n");
            }
            break;

        //step command
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
            auto_offset(i_motorcontrol);
            break;

        //set offset
        case 'o':
            i_motorcontrol.set_offset_value(value);
            printf("set offset to %d\n", value);
            break;

        //reverse torque
        case 'r':
            torque = -torque;
            i_motorcontrol.set_torque(torque);
            printf("Torque %d\n", torque);
            break;

        //enable and disable torque controller
        case 't':
            if (torque_control_flag == 0) {
                torque_control_flag = 1;
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");
            } else {
                torque_control_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control deactivated\n");
            }
            break;
        //set brake
        case 'b':
            if (brake_flag) {
                brake_flag = 0;
                printf("Brake blocking\n");
            } else {
                brake_flag = 1;
                printf("Brake released\n");
            }
            i_motorcontrol.set_brake_status(brake_flag);
            break;

        //set torque
        default:
            if (position_ctrl_flag) {
                position_ctrl_flag = 0;
                i_position_control.disable();
                delay_milliseconds(500);
                brake_flag = 1;
                i_motorcontrol.set_torque(0);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(brake_flag);
            }
            torque = value*sign;
            i_motorcontrol.set_torque(torque);
            printf("Torque %d\n", torque);
            break;
        }
        delay_milliseconds(10);
    }
}

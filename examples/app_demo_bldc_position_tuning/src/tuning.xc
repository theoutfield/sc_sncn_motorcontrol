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

    PosVelocityControlConfig pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();

    DownstreamControlData downstream_control_data;

    int int8_Kp_position = pos_velocity_ctrl_config.int10_P_position;
    int int8_Ki_position = pos_velocity_ctrl_config.int10_I_position;
    int int8_Kd_position = pos_velocity_ctrl_config.int10_D_position;
    int int16_P_error_limit_position = pos_velocity_ctrl_config.int21_P_error_limit_position;
    int int16_I_error_limit_position = pos_velocity_ctrl_config.int21_I_error_limit_position;
    int int16_integral_limit_position = pos_velocity_ctrl_config.int22_integral_limit_position;
    int int16_cmd_limit_position = pos_velocity_ctrl_config.int32_cmd_limit_position;

    int int8_Kp_velocity = pos_velocity_ctrl_config.int10_P_velocity;
    int int8_Ki_velocity = pos_velocity_ctrl_config.int10_I_velocity;//50;
    int int8_Kd_velocity = pos_velocity_ctrl_config.int10_D_velocity;
    int int16_P_error_limit_velocity = pos_velocity_ctrl_config.int21_P_error_limit_velocity;
    int int16_I_error_limit_velocity = pos_velocity_ctrl_config.int21_I_error_limit_velocity;
    int int16_integral_limit_velocity = pos_velocity_ctrl_config.int22_integral_limit_velocity;
    int int16_cmd_limit_velocity = pos_velocity_ctrl_config.int32_cmd_limit_velocity;

    int torque = 0;

//    delay_milliseconds(2000);
//    i_commutation.set_brake_status(1);

//    i_commutation.set_offset_value(3040);
    i_commutation.set_torque_control_enabled();
//    delay_milliseconds(1000);

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
            downstream_control_data.position_cmd = value*sign;
            i_position_control.update_control_data(downstream_control_data);
            printf("Go to %d\n", value*sign);
            break;

        //set velocity
        case 'v':
            downstream_control_data.velocity_cmd = value*sign;
            i_position_control.update_control_data(downstream_control_data);
            printf("set velocity %d\n", value*sign);
            break;

        //velocity pid coefficients
        case 'k':
            switch(mode_2) {
            case 'p':
                int8_Kp_velocity = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                break;
            case 'i':
                int8_Ki_velocity = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                break;
            case 'd':
                int8_Kd_velocity = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                break;
            default:
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_velocity, int8_Ki_velocity, int8_Kd_velocity);
                break;
            }
            break;
        //velocity pid limits
        case 'l':
            switch(mode_2) {
            case 'p':
                int16_P_error_limit_velocity = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                break;
            case 'i':
                int16_I_error_limit_velocity = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                break;
            case 'l':
                int16_integral_limit_velocity = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                break;
            case 'c':
                int16_cmd_limit_velocity = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                break;
            default:
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_velocity, int16_I_error_limit_velocity, int16_integral_limit_velocity, int16_cmd_limit_velocity);
                break;
            }
            break;

        //position pid coefficients
        case 'p':
            switch(mode_2) {
            case 'p':
                int8_Kp_position = value;
                i_position_control.set_position_pid_coefficients(int8_Kp_position, int8_Ki_position, int8_Kd_position);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_position, int8_Ki_position, int8_Kd_position);
                break;
            case 'i':
                int8_Ki_position = value;
                i_position_control.set_position_pid_coefficients(int8_Kp_position, int8_Ki_position, int8_Kd_position);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_position, int8_Ki_position, int8_Kd_position);
                break;
            case 'd':
                int8_Kd_position = value;
                i_position_control.set_position_pid_coefficients(int8_Kp_position, int8_Ki_position, int8_Kd_position);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_position, int8_Ki_position, int8_Kd_position);
                break;
            default:
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp_position, int8_Ki_position, int8_Kd_position);
                break;
            }
            break;
        //position pid limits
        case 'i':
            switch(mode_2) {
            case 'p':
                int16_P_error_limit_position = value * sign;
                i_position_control.set_position_pid_limits(int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                break;
            case 'i':
                int16_I_error_limit_position = value * sign;
                i_position_control.set_position_pid_limits(int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                break;
            case 'l':
                int16_integral_limit_position = value * sign;
                i_position_control.set_position_pid_limits(int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                break;
            case 'c':
                int16_cmd_limit_position = value * sign;
                i_position_control.set_position_pid_limits(int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
                break;
            default:
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit_position, int16_I_error_limit_position, int16_integral_limit_position, int16_cmd_limit_position);
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
                    downstream_control_data.velocity_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
//                    i_position_control.set_velocity(value*sign);
                    delay_milliseconds(200);
                    downstream_control_data.velocity_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(200);
                    downstream_control_data.velocity_cmd = 0;//value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 't':
                    printf("torque cmd: %d to %d (range:-32767 to 32767)\n", value*sign, -value*sign);
                    downstream_control_data.torque_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
//                    i_position_control.set_torque(value*sign);
                    delay_milliseconds(200);
                    downstream_control_data.torque_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
//                    i_position_control.set_torque(-value*sign);
                    delay_milliseconds(200);
                    downstream_control_data.torque_cmd = 0;
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
            i_position_control.disable();
            torque = value*sign;
            i_commutation.set_torque_control_enabled();
            i_commutation.set_torque(torque);
            printf("Torque %d\n", torque);
            break;
        }
        delay_milliseconds(10);
    }
}

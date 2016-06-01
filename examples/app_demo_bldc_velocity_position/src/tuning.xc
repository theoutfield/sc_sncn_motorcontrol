/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_commutation,
                       interface PositionControlInterface client ?i_position_control)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    int int8_Kp = 60;
    int int8_Ki = 35;
    int int8_Kd = 0;
    int int16_P_error_limit = 500;
    int int16_I_error_limit = 5;
    int int16_integral_limit = 200;
    int int16_cmd_limit = 400;

    delay_milliseconds(2000);

    i_position_control.set_velocity_pid_limits(int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
    i_position_control.set_velocity_pid_coefficients(int8_Kp, int8_Ki, int8_Kd);

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
            i_position_control.enable_position_ctrl();
            i_position_control.set_position(value*sign);
            printf("Go to %d (range:-32767 to 32767)\n", value*sign);
            delay_milliseconds(400);
            i_position_control.set_position(-value*sign);
            printf("Go to %d (range:-32767 to 32767)\n", -value*sign);
            delay_milliseconds(400);
            i_position_control.set_position(0);
//            i_position_control.disable_position_ctrl();
            printf("Go to %d (range:-32767 to 32767)\n", 0);
            break;
        //velocity pid coefficients
        case 'k':
            switch(mode_2) {
            case 'p':
                int8_Kp = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp, int8_Ki, int8_Kd);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp, int8_Ki, int8_Kd);
                break;
            case 'i':
                int8_Ki = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp, int8_Ki, int8_Kd);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp, int8_Ki, int8_Kd);
                break;
            case 'd':
                int8_Kd = value;
                i_position_control.set_velocity_pid_coefficients(int8_Kp, int8_Ki, int8_Kd);
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp, int8_Ki, int8_Kd);
                break;
            default:
                printf("Kp:%d Ki:%d Kd:%d\n", int8_Kp, int8_Ki, int8_Kd);
                break;
            }
            break;
        //velocity pid limits
        case 'l':
            switch(mode_2) {
            case 'p':
                int16_P_error_limit = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                break;
            case 'i':
                int16_I_error_limit = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                break;
            case 'l':
                int16_integral_limit = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                break;
            case 'c':
                int16_cmd_limit = value * sign;
                i_position_control.set_velocity_pid_limits(int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                break;
            default:
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", int16_P_error_limit, int16_I_error_limit, int16_integral_limit, int16_cmd_limit);
                break;
            }
            break;

        default:
            break;
        }
        delay_milliseconds(10);
    }
}

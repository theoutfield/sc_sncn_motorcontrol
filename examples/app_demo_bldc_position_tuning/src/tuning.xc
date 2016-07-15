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

    int offset = -1;
    while (offset == -1) {
        delay_milliseconds(50);//wait until offset is detected
        offset = i_motorcontrol.set_calib(0);
    }

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


void brake_shake(interface MotorcontrolInterface client i_motorcontrol, int torque) {
    const int period = 50;
    i_motorcontrol.set_brake_status(1);
    for (int i=0 ; i<1 ; i++) {
        i_motorcontrol.set_torque(torque);
        delay_milliseconds(period);
        i_motorcontrol.set_torque(-torque);
        delay_milliseconds(period);
    }
    i_motorcontrol.set_torque(0);
}

void run_offset_tuning(ProfilerConfig profiler_config,
                       client interface MotorcontrolInterface i_motorcontrol,
                       client interface PositionVelocityCtrlInterface ?i_position_control,
                       client interface PositionFeedbackInterface ?i_position_feedback)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    //variables
    int brake_flag = 0;
    int torque_control_flag = 0;
    int position_ctrl_flag = 0;
    int motor_polarity = 0, sensor_polarity = 0;
    int target_torque = 0;
//    int position_limit = 0;
    int status_mux = 0;
    //timing
    timer t;
    unsigned ts;
    t :> ts;
    //parameters structs
    MotorcontrolConfig motorcontrol_config = i_motorcontrol.get_config();
    PositionFeedbackConfig position_feedback_config;
    PosVelocityControlConfig pos_velocity_ctrl_config;
    if (!isnull(i_position_control)) {
        pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
    }
    DownstreamControlData downstream_control_data;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;
    downstream_control_data.position_cmd = 0;
    UpstreamControlData upstream_control_data;

    //brake and motorcontrol enable
    i_motorcontrol.set_brake_status(brake_flag);
    if (torque_control_flag == 1){
        i_motorcontrol.set_torque_control_enabled();
    }


    /* Initialise the position profile generator */
    if (!isnull(i_position_feedback)) {
        profiler_config.ticks_per_turn = i_position_feedback.get_ticks_per_turn();
        init_position_profiler(profiler_config);

        position_feedback_config = i_position_feedback.get_config();
        switch(position_feedback_config.sensor_type) {
        case BISS_SENSOR:
            sensor_polarity = position_feedback_config.biss_config.polarity;
            break;
        case CONTELEC_SENSOR:
            sensor_polarity = position_feedback_config.contelec_config.polarity;
            break;
        }
    }

    /* Initialise local variables */
    if (motorcontrol_config.polarity_type == NORMAL_POLARITY) {
        motor_polarity = 0;
    } else {
        motor_polarity = 1;
    }
//    if (!isnull(i_position_limiter)) {
//        position_limit = i_position_limiter.get_limit();
//    }

    fflush(stdout);
    //read and adjust the offset.
    while (1) {
        char mode = '@';
        char mode_2 = '@';
        char mode_3 = '@';
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
                if (mode == '@') {
                    mode = c;
                } else if (mode_2 == '@'){
                    mode_2 = c;
                } else {
                    mode_3 = c;
                }
            }
        }
        value *= sign;
        switch(mode) {
        //go to position directly
        case 'p':
            downstream_control_data.position_cmd = value;
            switch(mode_2) {
            case 'p':
                if (!isnull(i_position_feedback)) {
                    position_ctrl_flag = 1;
                    torque_control_flag = 0;
                    printf("Go to %d with profile\n", downstream_control_data.position_cmd);
                    set_profile_position(downstream_control_data, 1000, 1000, 1000, i_position_control);
                }
                break;
            default:
                i_position_control.update_control_data(downstream_control_data);
                printf("Go to %d\n", downstream_control_data.position_cmd);
                break;
            }
            break;

        //set velocity
        case 'v':
            downstream_control_data.velocity_cmd = value;
            downstream_control_data.position_cmd = downstream_control_data.velocity_cmd; //for display
            i_position_control.update_control_data(downstream_control_data);
            printf("set velocity %d\n", downstream_control_data.velocity_cmd);
            break;

        //pid coefficients
        case 'k':
            switch(mode_2) {
            case 'p': //position
                switch(mode_3) {
                case 'p':
                    pos_velocity_ctrl_config.int10_P_position = value;
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int10_I_position = value;
                    break;
                case 'd':
                    pos_velocity_ctrl_config.int10_D_position = value;
                    break;
                default:
                    printf("Pp:%d Pi:%d Pd:%d\n", pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position);
                    break;
                }
                break;
            case 'v': //velocity
                switch(mode_3) {
                case 'p':
                    pos_velocity_ctrl_config.int10_P_velocity = value;
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int10_I_velocity = value;
                    break;
                case 'd':
                    pos_velocity_ctrl_config.int10_D_velocity = value;
                    break;
                default:
                    printf("Kp:%d Ki:%d Kd:%d\n", pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity);
                    break;
                }
                break;
            }
            i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
            break;
        //limits
        case 'L':
            switch(mode_2) {
            case 'p': //position pid limits
                switch(mode_3) {
                case 'p':
                    pos_velocity_ctrl_config.int21_P_error_limit_position = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int21_I_error_limit_position = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed);
                    break;
                case 'l':
                    pos_velocity_ctrl_config.int22_integral_limit_position = value;
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
            case 'v': //velocity pid limits
                switch(mode_3) {
                case 'p':
                    pos_velocity_ctrl_config.int21_P_error_limit_velocity = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                    break;
                case 'i':
                    pos_velocity_ctrl_config.int21_I_error_limit_velocity = value;
                    i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                    printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                          , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                    break;
                case 'l':
                    pos_velocity_ctrl_config.int22_integral_limit_velocity = value;
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
            //max torque
            case 't':
                pos_velocity_ctrl_config.int21_max_torque = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("P_e_lim:%d I_e_lim:%d int_lim:%d cmd_lim:%d\n", pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity
                                                                      , pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque);
                break;
            //max speed
            case 's':
                pos_velocity_ctrl_config.int21_max_speed = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                break;
            }
            break;
        //step command
        case 'c':
            switch(mode_2) {
                case 'p':
                    printf("position cmd: %d to %d (range:-32767 to 32767)\n", value, -value);
                    downstream_control_data.position_cmd = value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.position_cmd = -value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.position_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 'v':
                    printf("velocity cmd: %d to %d (range:-32767 to 32767)\n", value, -value);
                    downstream_control_data.velocity_cmd = value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(500);
                    downstream_control_data.velocity_cmd = -value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(500);
                    downstream_control_data.velocity_cmd = 0;//value;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 't':
                    printf("torque cmd: %d to %d (range:-32767 to 32767)\n", value, -value);
                    downstream_control_data.torque_cmd = value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(400);
                    downstream_control_data.torque_cmd = -value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(400);
                    downstream_control_data.torque_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                case 'o':
                    printf("offset-torque cmd: %d to %d\n", value, -value);
                    downstream_control_data.position_cmd = 0;
                    downstream_control_data.velocity_cmd = 0;
                    downstream_control_data.offset_torque = value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(200);
                    downstream_control_data.offset_torque = -value;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(200);
                    downstream_control_data.offset_torque = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                    }
            break;

        //enable
        case 'e':
            if (value == 1) {
                switch(mode_2) {
                    case 'p':
                        position_ctrl_flag = 1;
                        torque_control_flag = 0;
                        downstream_control_data.position_cmd = upstream_control_data.position;
                        i_position_control.enable_position_ctrl();
                        printf("position ctrl enabled\n");
                        break;
                    case 'v':
                        position_ctrl_flag = 1;
                        torque_control_flag = 0;
                        downstream_control_data.velocity_cmd = 0;
                        downstream_control_data.position_cmd = downstream_control_data.velocity_cmd; //for display
                        i_position_control.enable_velocity_ctrl();
                        printf("velocity ctrl enabled\n");
                        break;
                    case 't':
                        position_ctrl_flag = 1;
                        torque_control_flag = 0;
                        i_position_control.enable_torque_ctrl();
                        printf("torque ctrl enabled\n");
                        break;
                }
            } else {
                position_ctrl_flag = 0;
                torque_control_flag = 0;
                brake_flag = 0;
                i_position_control.disable();
                printf("position ctrl disabled\n");
            }
            break;

        //pole pairs
        case 'P':
            if (!isnull(i_position_feedback)) {
                motorcontrol_config.pole_pair = value;
                position_feedback_config.biss_config.pole_pairs = value;
                position_feedback_config.contelec_config.pole_pairs = value;
                brake_flag = 0;
                torque_control_flag = 0;
                i_position_feedback.set_config(position_feedback_config);
                i_motorcontrol.set_config(motorcontrol_config);
            }
            break;

        //direction (motor polarity)
        case 'd':
            if (motorcontrol_config.polarity_type == NORMAL_POLARITY){
                motorcontrol_config.polarity_type = INVERTED_POLARITY;
                motor_polarity = 1;
            } else {
                motorcontrol_config.polarity_type = NORMAL_POLARITY;
                motor_polarity = 0;
            }
            i_motorcontrol.set_config(motorcontrol_config);
            torque_control_flag = 0;
            brake_flag = 0;
            break;

        //sensor polarity
        case 's':
            if (!isnull(i_position_feedback)) {
                if (sensor_polarity == 0) {
                    position_feedback_config.biss_config.polarity = 1;
                    position_feedback_config.contelec_config.polarity = 1;
                    sensor_polarity = 1;
                } else {
                    position_feedback_config.biss_config.polarity = 0;
                    position_feedback_config.contelec_config.polarity = 0;
                    sensor_polarity = 0;
                }
                i_position_feedback.set_config(position_feedback_config);
            }
            break;

        //position limiter
//        case 'l':
//            if (!isnull(i_position_limiter)) {
//                i_position_limiter.set_limit(value);
//                position_limit = i_position_limiter.get_limit();
//            }
//            break;

        //auto offset tuning
        case 'a':
            brake_flag = 1;
            i_motorcontrol.set_brake_status(brake_flag);
            motorcontrol_config.commutation_angle_offset = auto_offset(i_motorcontrol);
            break;

        //set offset
        case 'o':
            motorcontrol_config.commutation_angle_offset = value;
            i_motorcontrol.set_config(motorcontrol_config);
            brake_flag = 0;
            torque_control_flag = 0;
            printf("set offset to %d\n", value);
            break;

        //reverse torque
        case 'r':
            target_torque = -target_torque;
            i_motorcontrol.set_torque(target_torque);
            printf("Torque %d\n", target_torque);
            break;

        //enable and disable torque controller
        case 't':
            switch(mode_2) {
            case 's': //torque safe mode
                torque_control_flag = 0;
                i_motorcontrol.set_safe_torque_off_enabled();
                break;
            case 'o': //set torque offset
                downstream_control_data.offset_torque = value;
                break;
            default:
                if (torque_control_flag == 0 || value == 1) {
                    torque_control_flag = 1;
                    i_motorcontrol.set_torque_control_enabled();
                    printf("Torque control activated\n");
                } else {
                    torque_control_flag = 0;
                    i_motorcontrol.set_torque_control_disabled();
                    printf("Torque control deactivated\n");
                }
                break;
            }
            break;
        //set brake
        case 'b':
            switch(mode_2) {
            case 's':
                brake_flag = 1;
                brake_shake(i_motorcontrol, value);
                break;
            default:
                if (brake_flag == 0 || value == 1) {
                    brake_flag = 1;
                    printf("Brake released\n");
                } else {
                    brake_flag = 0;
                    printf("Brake blocking\n");
                }
                i_motorcontrol.set_brake_status(brake_flag);
                break;
            }
            break;

        //set zero position
        case 'z':
            if (!isnull(i_position_feedback)) {
//                    i_position_feedback.send_command(CONTELEC_CONF_NULL, 0, 0);
                i_position_feedback.send_command(CONTELEC_CONF_MTPRESET, value, 16);
                i_position_feedback.send_command(CONTELEC_CTRL_SAVE, 0, 0);
                i_position_feedback.send_command(CONTELEC_CTRL_RESET, 0, 0);
            }
            break;

        //set torque
        case '@':
            if (position_ctrl_flag) {
                position_ctrl_flag = 0;
                i_position_control.disable();
                delay_milliseconds(500);
                brake_flag = 1;
                torque_control_flag = 1;
                i_motorcontrol.set_torque(0);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(brake_flag);
            }
            target_torque = value;
            if (target_torque) {
                if (brake_flag == 0) {
                    brake_flag = 1;
                    i_motorcontrol.set_brake_status(brake_flag);
                }
                if (torque_control_flag == 0) {
                    torque_control_flag = 1;
                    i_motorcontrol.set_torque_control_enabled();
                }
            }
            i_motorcontrol.set_torque(target_torque);
            printf("Torque %d\n", target_torque);
            break;
        }
        delay_milliseconds(10);
    }
}

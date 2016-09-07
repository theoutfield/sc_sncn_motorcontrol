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

    while(i_motorcontrol.set_calib(0)==-1) delay_milliseconds(50);//wait until offset is detected

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


void run_offset_tuning(interface MotorcontrolInterface client i_motorcontrol,
                       interface PositionVelocityCtrlInterface client ?i_position_control)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");
    delay_milliseconds(1000);
    printf("\n");
    printf("g->print guiding manual\n");
    printf("------------------------------------------------------\n");
    printf(".Direct Torque Controller\n");
    printf("..b->brake                ..e->enable/disable\n");
    printf("..#->command torque       ..r->reverse torque\n");
    printf("..a->auto-tuning          ..m->sound\n");
    printf("..os#->set offset         ..op#->print offset\n");
    printf("------------------------------------------------------\n");
    printf(".Torque Controller\n");
    printf("..st1->set torque on      ..st0->set torque off\n");
    printf("..ct#->command torque back and forth\n");
    printf("------------------------------------------------------\n");
    printf(".Velocity Controller\n");
    printf("..kp#->Kp   ki#->Ki   kd#->Kd   kl#->integral_limit\n");
    printf("..sv#->set velocity on/off while # is the mode:\n");
    printf("...mode=1    -> VELOCITY_PID_CONTROLLER\n");
    printf("...mode=else -> disable\n");
    printf("..cv#->command velocity back and forth\n");
    printf("------------------------------------------------------\n");
    printf(".Position Controller\n");
    printf("..pp#->Kp   pi#->Ki   pd#->Kd   pl#->integral_limit\n");
    printf("..sp#->set pos on/off while # is the mode:\n");
    printf("...mode=1    -> POS_PID_CONTROLLER\n");
    printf("...mode=2    -> POS_PID_VELOCITY_CASCADED_CONTROLLER\n");
    printf("...mode=3    -> POS_INTEGRAL_OPTIMUM_CONTROLLER\n");
    printf("...mode=else -> disable\n");
    printf("..cp#->command position back and forth\n");
    printf("..cd#->command position\n");
    printf("------------------------------------------------------\n");
    printf(".Limits\n");
    printf("..lp#->pos_lim   lv#->velocity_lim   lt#->torque_lim\n");
    printf("------------------------------------------------------\n");
    printf("\n");

    DownstreamControlData downstream_control_data;
    PosVelocityControlConfig pos_velocity_ctrl_config;

    int torque = 0;
    int brake_flag = 0;
    int period_us;     // torque generation period in micro-seconds
    int pulse_counter; // number of generated pulses
    int torque_control_flag = 0;

    //profiler
    float pos_k = 0, pos_k_1n = 0, pos_k_2n = 0;
    float delta_T = 0.001;
    float a_max = 10000;
    float v_max = 5000;
    float pos_target;
    float pos_temp1, pos_temp2;
    int target_reached_flag = 0;
    float deceleration_distance = 0;
    float pos_deceleration = 0;
    int deceleration_flag = 0;

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

        case 'g':
            printf("\n");
            printf("g->print guiding manual\n");
            printf("------------------------------------------------------\n");
            printf(".Direct Torque Controller\n");
            printf("..b->brake                ..e->enable/disable\n");
            printf("..#->command torque       ..r->reverse torque\n");
            printf("..a->auto-tuning          ..m->sound\n");
            printf("..os#->set offset         ..op#->print offset\n");
            printf("------------------------------------------------------\n");
            printf(".Torque Controller\n");
            printf("..st1->set torque on      ..st0->set torque off\n");
            printf("..ct#->command torque back and forth\n");
            printf("------------------------------------------------------\n");
            printf(".Velocity Controller\n");
            printf("..kp#->Kp   ki#->Ki   kd#->Kd   kl#->integral_limit\n");
            printf("..sv#->set velocity on/off while # is the mode:\n");
            printf("...mode=1    -> VELOCITY_PID_CONTROLLER\n");
            printf("...mode=else -> disable\n");
            printf("..cv#->command velocity back and forth\n");
            printf("------------------------------------------------------\n");
            printf(".Position Controller\n");
            printf("..pp#->Kp   pi#->Ki   pd#->Kd   pl#->integral_limit\n");
            printf("..sp#->set pos on/off while # is the mode:\n");
            printf("...mode=1    -> POS_PID_CONTROLLER\n");
            printf("...mode=2    -> POS_PID_VELOCITY_CASCADED_CONTROLLER\n");
            printf("...mode=3    -> POS_INTEGRAL_OPTIMUM_CONTROLLER\n");
            printf("...mode=else -> disable\n");
            printf("..cp#->command position back and forth\n");
            printf("..cd#->command position\n");
            printf("------------------------------------------------------\n");
            printf(".Limits\n");
            printf("..lp#->pos_lim   lv#->velocity_lim   lt#->torque_lim\n");
            printf("------------------------------------------------------\n");
            printf("\n");
            break;
        //position pid coefficients
        case 'p':
            switch(mode_2) {
            case 'p':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.P_pos = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Pp:%d Pi:%d Pd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos,
                                                       pos_velocity_ctrl_config.D_pos, pos_velocity_ctrl_config.integral_limit_pos);
                break;
            case 'i':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.I_pos = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Pp:%d Pi:%d Pd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos,
                                                       pos_velocity_ctrl_config.D_pos, pos_velocity_ctrl_config.integral_limit_pos);
                break;
            case 'd':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.D_pos = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Pp:%d Pi:%d Pd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos,
                                                       pos_velocity_ctrl_config.D_pos, pos_velocity_ctrl_config.integral_limit_pos);
                break;
            case 'l':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.integral_limit_pos = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Pp:%d Pi:%d Pd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos,
                                                       pos_velocity_ctrl_config.D_pos, pos_velocity_ctrl_config.integral_limit_pos);
                break;
            default:
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                printf("Pp:%d Pi:%d Pd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos,
                                                       pos_velocity_ctrl_config.D_pos, pos_velocity_ctrl_config.integral_limit_pos);
                break;
            }
            break;

        //velocity pid coefficients
        case 'k':
            switch(mode_2) {
            case 'p':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.P_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                                       pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                break;
            case 'i':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.I_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                                       pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                break;
            case 'd':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.D_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                                       pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                break;
            case 'l':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.integral_limit_velocity = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                                       pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                break;
            default:
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                                       pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                break;
            }
            break;

        //limits
        case 'l':
            switch(mode_2) {
            case 'p':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.max_pos = value;
                pos_velocity_ctrl_config.min_pos = -value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("max_pos:%d max_v:%d max_torq:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.max_speed,
                                                            pos_velocity_ctrl_config.max_torque);
                break;
            case 'v':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.max_speed = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("max_pos:%d max_v:%d max_torq:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.max_speed,
                                                            pos_velocity_ctrl_config.max_torque);
                break;
            case 't':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                pos_velocity_ctrl_config.max_torque = value;
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("max_pos:%d max_v:%d max_torq:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.max_speed,
                                                            pos_velocity_ctrl_config.max_torque);
                break;
            default:
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                printf("max_pos:%d max_v:%d max_torq:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.max_speed,
                                                            pos_velocity_ctrl_config.max_torque);
                break;
            }
            break;

        case 's':
            switch(mode_2) {
                case 'p':
                    if (value == 1) {
                        i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
                        printf("position ctrl enabled\n");
                    }
                    else if (value == 2) {
                        i_position_control.enable_position_ctrl(POS_PID_VELOCITY_CASCADED_CONTROLLER);
                        printf("position ctrl enabled\n");
                    }
                    else if (value == 3) {
                        i_position_control.enable_position_ctrl(POS_INTEGRAL_OPTIMUM_CONTROLLER);
                        printf("position ctrl enabled\n");
                    }
                    else {
                        i_position_control.disable();
                        printf("position ctrl disabled\n");
                    }
                    break;
                case 'v':
                    if (value == 1) {
                        i_position_control.enable_velocity_ctrl(VELOCITY_PID_CONTROLLER);
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
                //command position forward and backward
                case 'p':
                    printf("position cmd: %d to %d\n", value*sign, -value*sign);
                    downstream_control_data.offset_torque = 0;
                    downstream_control_data.position_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1500);
                    downstream_control_data.position_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1500);
                    downstream_control_data.position_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                //command position direct
                case 'd':
                    printf("position cmd: %d\n", value*sign);
                    downstream_control_data.offset_torque = 0;
                    pos_target = value*sign;
                    //initial pos should be read from pos controller
                    pos_k_1n = pos_k;
                    pos_k_2n = pos_k;
                    target_reached_flag = 0;
//                    a_max = 10000;
//                    v_max = 5000;
                    deceleration_flag = 0;
                    while(target_reached_flag == 0) {
                        deceleration_distance = ((pos_k - pos_k_1n) / delta_T) * ((pos_k - pos_k_1n) / delta_T) / (2*a_max);// + a_max * delta_T * delta_T / 2;
                        pos_deceleration = pos_target - deceleration_distance;
                        if(pos_k >= pos_deceleration) {
                            target_reached_flag = 1;
                            printf("deceleration %d\n",(int)pos_k);
                        }
                        if(target_reached_flag == 0) {
                            pos_temp1 = (delta_T * delta_T * a_max) + (2 * pos_k_1n) - pos_k_2n; //sign of move should be added
                            pos_temp2 = (delta_T * v_max) + pos_k_1n; //sign of move should be added
                            if (pos_temp1 < pos_temp2)
                                pos_k = pos_temp1;
                            else
                                pos_k = pos_temp2;
                        }
                        else {
                            pos_k = (-delta_T * delta_T * a_max) + (2 * pos_k_1n) - pos_k_2n; //sign of move should be added
                            if (pos_k >= pos_target) {
                                pos_k = pos_target;
                                target_reached_flag = 1;
                            }
                        }
                        downstream_control_data.position_cmd = pos_k;
                        i_position_control.update_control_data(downstream_control_data);
                        pos_k_2n = pos_k_1n;
                        pos_k_1n = pos_k;
                        delay_microseconds(900);
                    }
                    break;
                //command velocity forward and backward
                case 'v':
                    printf("velocity cmd: %d to %d\n", value*sign, -value*sign);
                    downstream_control_data.offset_torque = 0;
                    downstream_control_data.velocity_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.velocity_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.velocity_cmd = 0;//value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                //command torque forward and backward
                case 't':
                    printf("torque cmd: %d to %d\n", value*sign, -value*sign);
                    downstream_control_data.torque_cmd = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.torque_cmd = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.torque_cmd = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                //command additive torque forward and backward
                case 'o':
                    printf("offset-torque cmd: %d to %d\n", value*sign, -value*sign);
                    downstream_control_data.position_cmd = 0;
                    downstream_control_data.velocity_cmd = 0;
                    downstream_control_data.offset_torque = value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.offset_torque = -value*sign;
                    i_position_control.update_control_data(downstream_control_data);
                    delay_milliseconds(1000);
                    downstream_control_data.offset_torque = 0;
                    i_position_control.update_control_data(downstream_control_data);
                    break;
                }
            break;

        //profiler max acceleration
        case 'j':
            a_max = (float) value;
            break;
        //profiler max velocity
        case 'x':
            v_max = (float) value;
            break;

        //auto offset tuning
        case 'a':
            auto_offset(i_motorcontrol);
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

        //offset
        case 'o':
            switch(mode_2) {
                //set offset
                case 's':
                    i_motorcontrol.set_offset_value(value);
                    printf("set offset to %d\n", value);
                    break;
                //print offset
                case 'p':
                    printf("offset %d\n", i_motorcontrol.set_calib(0));
                    break;
            }
            break;

        //enable and disable torque controller
        case 'e':
            if (torque_control_flag == 0) {
                torque_control_flag = 1;
                i_motorcontrol.set_brake_status(1);
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");
            } else {
                torque_control_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control deactivated\n");
            }
            break;

        //play sound!
        case 'm':
            for(period_us=400;period_us<=(1*1000);(period_us+=400))
            {
                if(period_us<3000) period_us-=300;

                for(pulse_counter=0;pulse_counter<=(50000/period_us);pulse_counter++)//total period = period * pulse_counter=1000000 us
                {
                    i_motorcontrol.set_torque(value);
                    delay_microseconds(period_us);
                    i_motorcontrol.set_torque(-value);
                    delay_microseconds(period_us);
                }
            }
            i_motorcontrol.set_torque(0);
            break;

        //reverse torque
        case 'r':
            torque = -torque;
            i_motorcontrol.set_torque(torque);
            printf("Torque %d [milli-Nm]\n", torque);
            break;

        //set torque
        default:
            torque = value * sign;
            i_motorcontrol.set_torque(torque);
            printf("torque %d [milli-Nm]\n", torque);
            break;
        }
        delay_milliseconds(10);
    }
}

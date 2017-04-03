/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

int auto_offset(interface MotorControlInterface client i_motorcontrol)
{
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    while(i_motorcontrol.get_offset()==-1) delay_milliseconds(50);//wait until offset is detected

    int offset=i_motorcontrol.get_offset();
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


void demo_torque_position_velocity_control(client interface PositionVelocityCtrlInterface i_position_control)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    DownstreamControlData downstream_control_data;
    MotionControlConfig motion_ctrl_config;

    MotorcontrolConfig motorcontrol_config;
    int proper_sensor_polarity=0;
    int offset=0;

    int velocity_running = 0;
    int velocity = 0;

    int torque = 0;
    int brake_flag = 0;
    int period_us;     // torque generation period in micro-seconds
    int pulse_counter; // number of generated pulses
    int torque_control_flag = 0;

    fflush(stdout);
    //read and adjust the offset.
    while (1)
    {
        char mode = '@';
        char mode_2 = '@';
        char mode_3 = '@';
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n')
        {
            if(isdigit(c)>0)
            {
                value *= 10;
                value += c - '0';
            }
            else if (c == '-')
            {
                sign = -1;
            }
            else if (c != ' ')
            {
                if (mode == '@')
                {
                    mode = c;
                }
                else if (mode_2 == '@')
                {
                    mode_2 = c;
                }
                else
                {
                    mode_3 = c;
                }
            }
        }
        value *= sign;

        switch(mode)
        {
        //position commands
        case 'p':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.position_cmd = value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                //direct command with profile
                case 'p':
                        //bug: the first time after one p# command p0 doesn't use the profile; only the way back to zero
                        motion_ctrl_config.enable_profiler = 1;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        printf("Go to %d with profile\n", value);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //step command (forward and backward)
                case 's':
                        switch(mode_3)
                        {
                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("position cmd: %d to %d with profile\n", value, -value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("position cmd: %d to %d\n", value, -value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.position_cmd = value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1500);
                        downstream_control_data.position_cmd = -value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1500);
                        downstream_control_data.position_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        motion_ctrl_config.enable_profiler = 0;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        printf("Go to %d\n", value);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                }
                break;

        //velocity commands
        case 'v':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.velocity_cmd = value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                //step command (forward and backward)
                case 's':
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.velocity_cmd = value;
                        switch(mode_3)
                        {
                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("velocity cmd: %d to %d with profile\n", value, -value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("velocity cmd: %d to %d\n", value, -value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.velocity_cmd = -value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.velocity_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        if(value==0)
                            velocity_running = 0;
                        else
                            velocity_running = 1;
                        downstream_control_data.offset_torque = 0;
                        velocity = value;
                        downstream_control_data.velocity_cmd = velocity;
                        i_position_control.update_control_data(downstream_control_data);
                        printf("set velocity %d\n", downstream_control_data.velocity_cmd);
                        break;
                }
                break;

        //enable and disable torque controller
        case 't':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.torque_cmd = value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                //step command (forward and backward)
                case 's':
                        downstream_control_data.torque_cmd = value;
                        switch(mode_3)
                        {
                        //torque safe mode
                        case 's':
                            i_position_control.set_safe_torque_off_enabled();
                            printf("torque safe enabled\n");
                            break;

                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("torque cmd: %d to %d with profile\n", value, -value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("torque cmd: %d to %d\n", value, -value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.torque_cmd = -value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.torque_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                case 'p':
                        downstream_control_data.torque_cmd = value;
                        motion_ctrl_config.enable_profiler = 1;
                        printf("torque cmd: %d with profile\n", value);
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.enable_profiler = 0;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.torque_cmd = value;
                        i_position_control.update_control_data(downstream_control_data);
                        printf("set torque %d\n", downstream_control_data.torque_cmd);
                        break;
                }
                break;

        //reverse torque
        case 'r':
                downstream_control_data.torque_cmd = -downstream_control_data.torque_cmd;
                i_position_control.update_control_data(downstream_control_data);
                printf("torque command %d milli-Nm\n", downstream_control_data.torque_cmd);
                break;

        //pid coefficients
        case 'k':
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                case 'p': //position
                        switch(mode_3)
                        {
                        case 'p':
                                motion_ctrl_config.position_kp = value;
                                break;
                        case 'i':
                                motion_ctrl_config.position_ki = value;
                                break;
                        case 'd':
                                motion_ctrl_config.position_kd = value;
                                break;
                        case 'l':
                                motion_ctrl_config.position_integral_limit = value;
                                break;
                        case 'j':
                                motion_ctrl_config.moment_of_inertia = value;
                                break;
                        default:
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        printf("Kp:%d Ki:%d Kd:%d j%d i_lim:%d\n",
                                motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd,
                                motion_ctrl_config.moment_of_inertia, motion_ctrl_config.position_integral_limit);
                        break;

                case 'v': //velocity
                        switch(mode_3)
                        {
                        case 'p':
                                motion_ctrl_config.velocity_kp = value;
                                break;
                        case 'i':
                                motion_ctrl_config.velocity_ki = value;
                                break;
                        case 'd':
                                motion_ctrl_config.velocity_kd = value;
                                break;
                        case 'l':
                                motion_ctrl_config.velocity_integral_limit = value;
                                break;
                        default:
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", motion_ctrl_config.velocity_kp, motion_ctrl_config.velocity_ki,
                                motion_ctrl_config.velocity_kd, motion_ctrl_config.velocity_integral_limit);
                        break;

                default:
                        printf("kp->pos_ctrl ko->optimum_ctrl kv->vel_ctrl\n");
                        break;
                }

                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                break;

        //limits
        case 'L':
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                //max position limit
                case 'p':
                    switch(mode_3)
                    {
                    case 'u':
                        motion_ctrl_config.max_pos_range_limit = value;
                        break;
                    case 'l':
                        motion_ctrl_config.min_pos_range_limit = value;
                        break;
                    default:
                        motion_ctrl_config.max_pos_range_limit = value;
                        motion_ctrl_config.min_pos_range_limit = -value;
                        break;
                    }
                    break;

                //max velocity limit
                case 'v':
                        motion_ctrl_config.max_motor_speed = value;
                        break;

                //max torque limit
                case 't':
                        motion_ctrl_config.max_torque = value;
                        break;

                default:
                        break;
                }
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                printf("pos_max:%d pos_min:%d v_max:%d torq_max:%d\n", motion_ctrl_config.max_pos_range_limit, motion_ctrl_config.min_pos_range_limit, motion_ctrl_config.max_motor_speed,
                        motion_ctrl_config.max_torque);
                break;

        //change direction/polarity of the movement in position/velocity control
        case 'd':
            motion_ctrl_config = i_position_control.get_position_velocity_control_config();
            if (motion_ctrl_config.polarity == -1)
            {
                motion_ctrl_config.polarity = 1;
                printf("normal movement polarity\n");
            }
            else
            {
                motion_ctrl_config.polarity = -1;
                printf("inverted movement polarity\n");
            }
            i_position_control.set_position_velocity_control_config(motion_ctrl_config);
            break;

        //enable
        case 'e':
                switch(mode_2)
                {
                case 'p':
                        if (value == 1)
                        {
                            i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
                            printf("simple PID pos ctrl enabled\n");
                        }
                        else if (value == 2)
                        {
                            i_position_control.enable_position_ctrl(POS_PID_VELOCITY_CASCADED_CONTROLLER);
                            printf("vel.-cascaded pos ctrl enabled\n");
                        }
                        else if (value == 3)
                        {
                            i_position_control.enable_position_ctrl(NL_POSITION_CONTROLLER);
                            printf("Nonlinear pos ctrl enabled\n");
                        }
                        else
                        {
                            i_position_control.disable();
                            printf("position ctrl disabled\n");
                        }
                        break;
                case 'v':
                        if (value == 1)
                        {
                            i_position_control.enable_velocity_ctrl();
                            printf("velocity ctrl enabled\n");
                        }
                        else
                        {
                            i_position_control.disable();
                            printf("velocity ctrl disabled\n");
                        }
                        break;
                case 't':
                        if (value == 1)
                        {
                            torque_control_flag = 1;
                            i_position_control.enable_torque_ctrl();
                            printf("torque ctrl enabled\n");
                        }
                        else
                        {
                            torque_control_flag = 0;
                            i_position_control.disable();
                            printf("torque ctrl disabled\n");
                        }
                        break;
                default:
                        printf("ep1->enable PID pos ctrl\n");
                        printf("ep2->enable cascaded pos ctrl\n");
                        printf("ep3->enable integral-optimum pos ctrl\n");
                        printf("ev1->enable PID velocity ctrl\n");
                        printf("et1->enable torque ctrl\n");
                        break;
                }
                break;
        //help
        case 'h':
                printf("p->set position\n");
                printf("v->set veloctiy\n");
                printf("k->set PIDs\n");
                printf("L->set limits\n");
                printf("e->enable controllers\n");
                break;

        //jerk limitation (profiler parameters)
        case 'j':
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                case 'a':
                        motion_ctrl_config.max_acceleration_profiler = value;
                        break;
                case 'd':
                        motion_ctrl_config.max_deceleration_profiler = value;
                        break;
                case 'v':
                        motion_ctrl_config.max_speed_profiler = value;
                        break;
                case 't':
                        motion_ctrl_config.max_torque_rate_profiler = value;
                        break;

                default:
                        break;
                }
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                printf("profiler settings: \n");
                printf("acceleration: %d [rpm/s], deceleration: %d \n velocity: %d [rpm], torque_rate: %d [mNm/s] \n", motion_ctrl_config.max_acceleration_profiler, motion_ctrl_config.max_deceleration_profiler, motion_ctrl_config.max_speed_profiler, motion_ctrl_config.max_torque_rate_profiler);
                break;

        //auto offset tuning
        case 'a':
                printf("Sending offset_detection command ...\n");

                motorcontrol_config = i_position_control.set_offset_detection_enabled();

                if(motorcontrol_config.commutation_angle_offset == -1)
                {
                    printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
                }
                else
                {
                    motorcontrol_config = i_position_control.get_motorcontrol_config();
                    printf(">>  PROPER POSITION SENSOR POLARITY ...\n");

                    printf("Detected offset is: %i\n", motorcontrol_config.commutation_angle_offset);

                    if(motorcontrol_config.commutation_sensor==HALL_SENSOR)
                    {
                        printf("SET THE FOLLOWING CONSTANTS IN CASE OF LOW-QUALITY HALL SENSOR \n");
                        for (int i=0;i<6;i++) {
                            printf("      hall_state_angle[%d]: %d\n", i, motorcontrol_config.hall_state[i]);
                        }
                    }
                }
                break;

        //set brake
        case 'b':
                switch(mode_2)
                {
                case 's':
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.brake_release_strategy = value;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        break;

                case 'v'://brake voltage configure
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        switch(mode_3)
                        {
                        case 'n':// nominal voltage of dc-bus
                                // set
                                motion_ctrl_config.dc_bus_voltage=value;
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                // check
                                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                                i_position_control.update_brake_configuration();
                                printf("nominal voltage of dc-bus is %d Volts \n", motion_ctrl_config.dc_bus_voltage);
                                break;

                        case 'p':// pull voltage for releasing the brake at startup
                                //set
                                motion_ctrl_config.pull_brake_voltage=value;
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                // check
                                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                                i_position_control.update_brake_configuration();
                                printf("brake pull voltage is %d milli-Volts \n", motion_ctrl_config.pull_brake_voltage);
                                break;

                        case 'h':// hold voltage for holding the brake after it is pulled
                                //set
                                motion_ctrl_config.hold_brake_voltage=value;
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                // check
                                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                i_position_control.update_brake_configuration();
                                printf("brake hold voltage is %d milli-Volts\n", motion_ctrl_config.hold_brake_voltage);
                                break;
                        default:
                                break;
                        }
                        break;

                case 't'://set pull time
                        //set
                        motion_ctrl_config.pull_brake_time=value;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        // check
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        i_position_control.update_brake_configuration();
                        printf("brake pull time is %d milli-seconds \n", motion_ctrl_config.pull_brake_time);
                        break;

                default:
                        if (brake_flag)
                        {
                            brake_flag = 0;
                            printf("Brake blocking\n");
                        }
                        else
                        {
                            brake_flag = 1;
                            printf("Brake released\n");
                        }
                        i_position_control.set_brake_status(brake_flag);
                        break;
                }
                break;

        //set offset
        case 'o':
                motorcontrol_config = i_position_control.get_motorcontrol_config();
                switch(mode_2)
                {
                //set offset
                case 's':
                    motorcontrol_config.commutation_angle_offset = value;
                    i_position_control.set_motorcontrol_config(motorcontrol_config);
                    printf("set offset to %d\n", motorcontrol_config.commutation_angle_offset);
                    break;
                //print offset
                case 'p':
                    printf("offset %d\n", motorcontrol_config.commutation_angle_offset);
                    break;
                }
                break;

        //check status and reset faults if any
        case 'f':
            UpstreamControlData upstream_control_data = i_position_control.update_control_data(downstream_control_data);

            if (upstream_control_data.error_status == NO_FAULT)
            {
                printf("No fault\n");
            }
            else
            {
                //disable position and motorcontrol
                i_position_control.disable();
                brake_flag = 0;

                if(upstream_control_data.error_status != NO_FAULT)
                    printf(">>  FAULT ID %i DETECTED ...\n", upstream_control_data.error_status);

                //reset fault
                printf("Reset fault...\n");
                i_position_control.reset_motorcontrol_faults();

                //check if reset worked
                delay_milliseconds(500);
                upstream_control_data = i_position_control.update_control_data(downstream_control_data);
                if(upstream_control_data.error_status == NO_FAULT)
                {
                    printf(">>  FAULT REMOVED\n");
                } else {
                    printf(">>  FAULT ID %i NOT REMOVED!\n");
                }
            }
            break;

        //disable controllers
        default:
                i_position_control.disable();
                printf("controller disabled\n");
                break;

        }
        delay_milliseconds(10);
    }
}

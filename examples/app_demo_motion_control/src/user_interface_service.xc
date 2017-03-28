/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <user_interface_service.h>
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


/**
 * @brief receive user inputs from xtimecomposer. By default, it gets 3 characters, and one value from the console.
 *
 * @return ConsoleInputs structure including the user inputs.
 */
ConsoleInputs get_user_command()
{
    ConsoleInputs console_inputs;
    console_inputs.first_char  = '0';
    console_inputs.second_char = '0';
    console_inputs.third_char  = '0';
    console_inputs.value       = 0;


    char mode_1 = '@';
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
            if (mode_1 == '@')
            {
                mode_1 = c;
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

    console_inputs.first_char  = mode_1;
    console_inputs.second_char = mode_2;
    console_inputs.third_char  = mode_3;
    console_inputs.value = value;

    return console_inputs;
}

/**
 * @brief Demonstrate usage of:
 *      - position controller (with a simple profiler)
 *      - velocity controller (with a simple profiler)
 *      - torque controller   (bypassing higher level controllers, with a simple profiler)
 *
 * @return ConsoleInputs structure including the user inputs.
 */
void demo_motion_control(client interface PositionVelocityCtrlInterface i_position_control)
{
    delay_milliseconds(1000);
    printf(">> SOMANET MOTION CONTROL SERVICE STARTING ...\n");

    DownstreamControlData downstream_control_data;
    MotionControlConfig motion_ctrl_config;
    MotorcontrolConfig motorcontrol_config;

    ConsoleInputs console_inputs;
    console_inputs.first_char  = '0';
    console_inputs.second_char = '0';
    console_inputs.third_char  = '0';
    console_inputs.value       =  0 ;


    int brake_flag = 0;

    fflush(stdout);
    //read and adjust the offset.
    while (1)
    {

    while (1)
    {

        printf("\n>> please select your motion control mode ...\n");
        printf(">> press t for torque   control mode\n");
        printf(">> press v for velocity control mode\n");
        printf(">> press p for position control mode\n");
        printf(">> press q to quit\n");

        console_inputs = get_user_command();
        while(console_inputs.first_char!='t' && console_inputs.first_char!='v' && console_inputs.first_char!='p' && console_inputs.first_char!='q')
        {
            printf("wrong input\n");
            console_inputs = get_user_command();
        }

        switch(console_inputs.first_char)
        {
        case 't':
                i_position_control.enable_torque_ctrl();
                printf("torque control mode with linear profil selected\n");

                printf("please enter the reference torque in milli-Nm\n");
                console_inputs = get_user_command();

                printf("torque step command from 0 to %d to %d to 0 milli-Nm\n", console_inputs.value, -console_inputs.value);

                downstream_control_data.offset_torque = 0;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                motion_ctrl_config.enable_profiler = 1;
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);

                downstream_control_data.torque_cmd = console_inputs.value;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                downstream_control_data.torque_cmd =-console_inputs.value;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                downstream_control_data.torque_cmd =0;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                i_position_control.disable();
                printf("torque controller disabled\n");

                break;

        case 'v':
                i_position_control.enable_velocity_ctrl();
                printf("velocity control mode with linear profil selected\n");

                printf("please enter the reference velocity in rpm\n");
                console_inputs = get_user_command();

                printf("velocity step command from 0 to %d to %d to 0 rpm\n", console_inputs.value, -console_inputs.value);

                downstream_control_data.offset_torque = 0;
                downstream_control_data.velocity_cmd = console_inputs.value;

                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                motion_ctrl_config.enable_profiler = 1;
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);

                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                downstream_control_data.velocity_cmd = -console_inputs.value;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                downstream_control_data.velocity_cmd = 0;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(2000);

                i_position_control.disable();
                printf("velocity controller disabled\n");
                break;

        case 'p':
                printf("please select position controller type\n");
                printf("press 1 to use a position pid controller \n");
                printf("press 2 to use a cascaded positon/velocity pid controller \n");
                printf("press 3 to use a nonlinear position controller \n");

                console_inputs = get_user_command();
                while(console_inputs.value!=1 && console_inputs.value!=2 && console_inputs.value!=3)
                {
                    printf("wrong input\n");
                    printf("press 1 to use a position pid controller \n");
                    printf("press 2 to use a cascaded positon/velocity pid controller \n");
                    printf("press 3 to use a nonlinear position controller \n");

                    console_inputs = get_user_command();
                }

                switch(console_inputs.value)
                {
                case 1:
                        i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
                        printf("position pid controller with linear profiler selected\n");
                        break;

                case 2:
                        i_position_control.enable_position_ctrl(POS_PID_VELOCITY_CASCADED_CONTROLLER);
                        printf("cascaded position/velocity controller with linear profiler selected\n");
                        break;

                case 3:
                        i_position_control.enable_position_ctrl(NL_POSITION_CONTROLLER);
                        printf("nonlinear position controller with linear profiler selected\n");
                        break;

                default:
                        break;
                }

                printf("please enter the reference position\n");
                console_inputs = get_user_command();

                downstream_control_data.offset_torque = 0;
                downstream_control_data.position_cmd = console_inputs.value;

                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                motion_ctrl_config.enable_profiler = 1;
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);

                printf("position step command from 0 to %d to %d to 0\n", console_inputs.value, -console_inputs.value);

                downstream_control_data.offset_torque = 0;
                downstream_control_data.position_cmd = console_inputs.value;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(1500);
                downstream_control_data.position_cmd = -console_inputs.value;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(1500);
                downstream_control_data.position_cmd = 0;
                i_position_control.update_control_data(downstream_control_data);
                delay_milliseconds(1500);
                break;

        case 'q':
                i_position_control.disable();
                printf(" selected to quit\n");
                break;

        default:
                break;
        }

    }

        while(1);



































































        while(1);

        switch(console_inputs.first_char)
        {
        //position commands
        case 'p':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.position_cmd = console_inputs.value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(console_inputs.second_char)
                {
                //direct command with profile
                case 'p':
                        //bug: the first time after one p# command p0 doesn't use the profile; only the way back to zero
                        motion_ctrl_config.enable_profiler = 1;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        printf("Go to %d with profile\n", console_inputs.value);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //step command (forward and backward)
                case 's':
                        switch(console_inputs.third_char)
                        {
                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("position cmd: %d to %d with profile\n", console_inputs.value, -console_inputs.value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("position cmd: %d to %d\n", console_inputs.value, -console_inputs.value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.position_cmd = console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1500);
                        downstream_control_data.position_cmd = -console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1500);
                        downstream_control_data.position_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        motion_ctrl_config.enable_profiler = 0;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        printf("Go to %d\n", console_inputs.value);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                }
                break;


        //velocity commands
        case 'v':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.velocity_cmd = console_inputs.value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(console_inputs.second_char)
                {
                //step command (forward and backward)
                case 's':
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.velocity_cmd = console_inputs.value;
                        switch(console_inputs.third_char)
                        {
                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("velocity cmd: %d to %d with profile\n", console_inputs.value, -console_inputs.value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("velocity cmd: %d to %d\n", console_inputs.value, -console_inputs.value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.velocity_cmd = -console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.velocity_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.velocity_cmd = console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        printf("set velocity %d\n", downstream_control_data.velocity_cmd);
                        break;
                }
                break;





        //enable and disable torque controller
        case 't':
                downstream_control_data.offset_torque = 0;
                downstream_control_data.torque_cmd = console_inputs.value;
                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(console_inputs.second_char)
                {
                //step command (forward and backward)
                case 's':
                        downstream_control_data.torque_cmd = console_inputs.value;
                        switch(console_inputs.third_char)
                        {
                        //with profile
                        case 'p':
                                motion_ctrl_config.enable_profiler = 1;
                                printf("torque cmd: %d to %d with profile\n", console_inputs.value, -console_inputs.value);
                                break;
                        //without profile
                        default:
                                motion_ctrl_config.enable_profiler = 0;
                                printf("torque cmd: %d to %d\n", console_inputs.value, -console_inputs.value);
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.torque_cmd = -console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        delay_milliseconds(1000);
                        downstream_control_data.torque_cmd = 0;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                case 'p':
                        downstream_control_data.torque_cmd = console_inputs.value;
                        motion_ctrl_config.enable_profiler = 1;
                        printf("torque cmd: %d with profile\n", console_inputs.value);
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                //direct command
                default:
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.enable_profiler = 0;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.torque_cmd = console_inputs.value;
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
                switch(console_inputs.second_char)
                {
                case 'p': //position
                        switch(console_inputs.third_char)
                        {
                        case 'p':
                                motion_ctrl_config.position_kp = console_inputs.value;
                                break;
                        case 'i':
                                motion_ctrl_config.position_ki = console_inputs.value;
                                break;
                        case 'd':
                                motion_ctrl_config.position_kd = console_inputs.value;
                                break;
                        case 'l':
                                motion_ctrl_config.position_integral_limit = console_inputs.value;
                                break;
                        case 'j':
                                motion_ctrl_config.moment_of_inertia = console_inputs.value;
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
                        switch(console_inputs.third_char)
                        {
                        case 'p':
                                motion_ctrl_config.velocity_kp = console_inputs.value;
                                break;
                        case 'i':
                                motion_ctrl_config.velocity_ki = console_inputs.value;
                                break;
                        case 'd':
                                motion_ctrl_config.velocity_kd = console_inputs.value;
                                break;
                        case 'l':
                                motion_ctrl_config.velocity_integral_limit = console_inputs.value;
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
                switch(console_inputs.second_char)
                {
                //max position limit
                case 'p':
                    switch(console_inputs.third_char)
                    {
                    case 'u':
                        motion_ctrl_config.max_pos_range_limit = console_inputs.value;
                        break;
                    case 'l':
                        motion_ctrl_config.min_pos_range_limit = console_inputs.value;
                        break;
                    default:
                        motion_ctrl_config.max_pos_range_limit =  console_inputs.value;
                        motion_ctrl_config.min_pos_range_limit = -console_inputs.value;
                        break;
                    }
                    break;

                //max velocity limit
                case 'v':
                        motion_ctrl_config.max_motor_speed = console_inputs.value;
                        break;

                //max torque limit
                case 't':
                        motion_ctrl_config.max_torque = console_inputs.value;
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
                switch(console_inputs.second_char)
                {
                case 'p':
                        if (console_inputs.value == 1)
                        {
                            i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
                            printf("simple PID pos ctrl enabled\n");
                        }
                        else if (console_inputs.value == 2)
                        {
                            i_position_control.enable_position_ctrl(POS_PID_VELOCITY_CASCADED_CONTROLLER);
                            printf("vel.-cascaded pos ctrl enabled\n");
                        }
                        else if (console_inputs.value == 3)
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
                        if (console_inputs.value == 1)
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
                        if (console_inputs.value == 1)
                        {
                            i_position_control.enable_torque_ctrl();
                            printf("torque ctrl enabled\n");
                        }
                        else
                        {
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
                switch(console_inputs.second_char)
                {
                case 'a':
                        motion_ctrl_config.max_acceleration_profiler = console_inputs.value;
                        break;
                case 'v':
                        motion_ctrl_config.max_speed_profiler = console_inputs.value;
                        break;
                case 't':
                        motion_ctrl_config.max_torque_rate_profiler = console_inputs.value;
                        break;

                default:
                        break;
                }
                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                printf("profiler settings: \n");
                printf("acceleration: %d [rpm/s], velocity: %d [rpm], torque_rate: %d [mNm/s] \n",motion_ctrl_config.max_acceleration_profiler, motion_ctrl_config.max_speed_profiler, motion_ctrl_config.max_torque_rate_profiler);
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
                switch(console_inputs.second_char)
                {
                case 's':
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.special_brake_release = console_inputs.value;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                        break;

                case 'v'://brake voltage configure
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        switch(console_inputs.third_char)
                        {
                        case 'n':// nominal voltage of dc-bus
                                // set
                                motion_ctrl_config.dc_bus_voltage=console_inputs.value;
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                // check
                                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                                i_position_control.update_brake_configuration();
                                printf("nominal voltage of dc-bus is %d Volts \n", motion_ctrl_config.dc_bus_voltage);
                                break;

                        case 'p':// pull voltage for releasing the brake at startup
                                //set
                                motion_ctrl_config.pull_brake_voltage=console_inputs.value;
                                i_position_control.set_position_velocity_control_config(motion_ctrl_config);
                                // check
                                motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                                i_position_control.update_brake_configuration();
                                printf("brake pull voltage is %d milli-Volts \n", motion_ctrl_config.pull_brake_voltage);
                                break;

                        case 'h':// hold voltage for holding the brake after it is pulled
                                //set
                                motion_ctrl_config.hold_brake_voltage=console_inputs.value;
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
                        motion_ctrl_config.pull_brake_time=console_inputs.value;
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
                switch(console_inputs.second_char)
                {
                //set offset
                case 's':
                    motorcontrol_config.commutation_angle_offset = console_inputs.value;
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
        case 'z':
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

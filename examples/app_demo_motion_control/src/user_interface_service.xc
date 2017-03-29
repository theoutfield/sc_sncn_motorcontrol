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

    char control_mode = 'q'; //'t' stands for torque control mode, 'v' stands for velocity control mode, 'p' stands for position control mode
    int repeat_mode = 0;


    fflush(stdout);
    while (1)
    {

        if(control_mode == 'q')
        {
            printf("\n>> please select your motion control mode ...\n");
            printf(">> press t for torque   control mode\n");
            printf(">> press v for velocity control mode\n");
            printf(">> press p for position control mode\n");

            console_inputs = get_user_command();
            while(console_inputs.first_char!='t' && console_inputs.first_char!='v' && console_inputs.first_char!='p')
            {
                printf("wrong input\n");

                printf(">> press t for torque   control mode\n");
                printf(">> press v for velocity control mode\n");
                printf(">> press p for position control mode\n");

                console_inputs = get_user_command();
            }
            control_mode = console_inputs.first_char;
        }

        switch(control_mode)
        {
        case 't':
                if(repeat_mode==0)
                {
                    i_position_control.enable_torque_ctrl();
                    printf("torque control mode with linear profil selected\n");
                }

                int command_type = 'd';// 's' stands for step response with profiler, and 'd' stands for direct command

                printf("please select command type type\n");
                printf("press s to send a step command \n");
                printf("press d to send a direct command \n");

                console_inputs = get_user_command();
                while(console_inputs.first_char!='s' && console_inputs.first_char!='d')
                {
                    printf("wrong input\n");
                    printf("press s to send a step command \n");
                    printf("press d to send a direct command \n");

                    console_inputs = get_user_command();
                }
                command_type = console_inputs.first_char;

                switch(command_type)
                {
                case 's':
                        printf("step command selected\n");

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
                        break;

                case 'd':

                        printf("direct command selected\n");

                        printf("please enter the reference torque in milli-Nm\n");
                        console_inputs = get_user_command();

                        printf("torque step command set to %d milli-Nm\n", console_inputs.value, -console_inputs.value);

                        downstream_control_data.offset_torque = 0;
                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.enable_profiler = 1;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);

                        downstream_control_data.torque_cmd = console_inputs.value;
                        i_position_control.update_control_data(downstream_control_data);
                        break;
                }

                printf("press q if you want to quit this mode, else press any key\n");
                repeat_mode=1;
                console_inputs = get_user_command();
                if(console_inputs.first_char=='q')
                {
                    control_mode='q';
                    i_position_control.disable();
                    repeat_mode=0;
                    printf("controller disabled\n");
                }

                break;

        case 'v':
                if(repeat_mode==0)
                {
                    i_position_control.enable_velocity_ctrl();
                    printf("velocity control mode with linear profile selected\n");
                }
                int command_type = 'd';// 's' stands for step response with profiler, and 'd' stands for direct command

                printf("please select command type type\n");
                printf("press s to send a step command \n");
                printf("press d to send a direct command \n");

                console_inputs = get_user_command();
                while(console_inputs.first_char!='s' && console_inputs.first_char!='d')
                {
                    printf("wrong input\n");
                    printf("press s to send a step command \n");
                    printf("press d to send a direct command \n");

                    console_inputs = get_user_command();
                }
                command_type = console_inputs.first_char;

                switch(command_type)
                {
                case 's':
                        printf("step command selected\n");
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
                        break;
                case 'd':
                        printf("direct command selected\n");
                        printf("please enter the reference velocity in rpm\n");
                        console_inputs = get_user_command();

                        printf("velocity step command set to %d rpm\n", console_inputs.value);

                        downstream_control_data.offset_torque = 0;
                        downstream_control_data.velocity_cmd = console_inputs.value;

                        motion_ctrl_config = i_position_control.get_position_velocity_control_config();
                        motion_ctrl_config.enable_profiler = 1;
                        i_position_control.set_position_velocity_control_config(motion_ctrl_config);

                        i_position_control.update_control_data(downstream_control_data);
                        break;
                }

                printf("press q if you want to quit this mode, else press any key\n");
                repeat_mode=1;
                console_inputs = get_user_command();
                if(console_inputs.first_char=='q')
                {
                    control_mode='q';
                    i_position_control.disable();
                    repeat_mode=0;
                    printf("controller disabled\n");
                }

                break;

        case 'p':

                int controller_type = 1;
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

                controller_type = console_inputs.value;

                switch(controller_type)
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

                int command_type = 'd';// 's' stands for step response with profiler, and 'd' stands for direct command

                printf("please select command type type\n");
                printf("press s to send a step command \n");
                printf("press d to send a direct command \n");

                console_inputs = get_user_command();
                while(console_inputs.first_char!='s' && console_inputs.first_char!='d')
                {
                    printf("wrong input\n");
                    printf("press s to send a step command \n");
                    printf("press d to send a direct command \n");

                    console_inputs = get_user_command();
                }
                command_type = console_inputs.first_char;

                switch(command_type)
                {
                case 's':
                        printf("step command selected\n");
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

                case 'd':
                        printf("direct command selected\n");
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
                        break;
                }

                printf("press q if you want to quit this mode, else press any key\n");
                repeat_mode=1;
                console_inputs = get_user_command();
                if(console_inputs.first_char=='q')
                {
                    control_mode='q';
                    i_position_control.disable();
                    repeat_mode=0;
                    printf("controller disabled\n");
                }

                break;

        default:
                break;
        }

    }

}

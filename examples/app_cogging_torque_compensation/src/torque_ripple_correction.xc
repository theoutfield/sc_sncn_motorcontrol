/*
 * torque_ripple_correction.xc
 *
 *  Created on: Mar 20, 2017
 *      Author: jgofre
 */

#include <torque_ripple_correction.h>
#include <stdio.h>
#include <data_processing.h>

int cogging_torque [2][STEPS_PER_ROTATION] = {{0}};

//int friction_constant;

void interpolate_sensor_data (short int *array_in, short int * is_recorded, int sensor_resolution)
{
    int x_inf, y_inf, x_sup, y_sup;
    for (int i = 0 ; i < SENSOR_RESOLUTION; i++)
    {
        if (is_recorded[i])
        {
            array_in [i] = array_in [i]/is_recorded[i];
        }
    }
    for (int i = 0 ; i < SENSOR_RESOLUTION; i++)
    {
        if (!is_recorded[i])
        {
            int index_inf;
            if (i != 0)
            {
                index_inf = i-1;
                x_inf = index_inf;
            }
            else
            {
                index_inf = SENSOR_RESOLUTION;
                //Remember that for a sensor the last data of the array is really close to the first
                //Go backwards through all the possible positions until we find one that is recorded
                while (!is_recorded[--index_inf]);
                x_inf = index_inf-SENSOR_RESOLUTION;

            }
            int index_sup = i;
            //Go through all the possible positions until we find one that is recorded greater than the current one
            if (index_sup != SENSOR_RESOLUTION-1)
            {
                while (!is_recorded[++index_sup] && index_sup < SENSOR_RESOLUTION - 1);
            }
            x_sup = index_sup;

            if(!is_recorded[index_sup])
            {
                index_sup = 0;
                x_sup = SENSOR_RESOLUTION;
            }


            y_inf = array_in[index_inf];
            y_sup = array_in[index_sup];

            double tmp_value = (double)(i-x_inf)*(double)(y_sup-y_inf);
            array_in[i] = y_inf+tmp_value/(x_sup-x_inf);
        }
    }
}

int interpolate_sensor_torque (int sensor_position)
{
    int x_inf, y_inf, x_sup = 0, y_sup;
    int interpolated_torque;

    if ( !sensor_position )
    {
        interpolated_torque = cogging_torque[1][0];
    }
    else
    {
        while(cogging_torque[0][x_sup] < sensor_position && x_sup < STEPS_PER_ROTATION - 1)
        {
            x_sup++;
        }

        if (cogging_torque[0][x_sup] < sensor_position)
        {
            x_sup = 0;
        }
        if (x_sup == 0)
        {
            x_inf = STEPS_PER_ROTATION - 1;
        }
        else
        {
            x_inf = x_sup - 1;
        }
        y_inf = cogging_torque[1][x_inf];
        y_sup = cogging_torque[1][x_sup];

        x_inf = cogging_torque[0][x_inf];
        x_sup = cogging_torque[0][x_sup];

        if (x_sup == 0)
        {
            x_sup += SENSOR_RESOLUTION;
        }

        double tmp_value = (double)(sensor_position-x_inf)*(double)(y_sup-y_inf);
        interpolated_torque = y_inf+tmp_value/(x_sup-x_inf);
    }


    return interpolated_torque;
}

int remove_friction_torque()
{
    float torque_average = 0, steps = STEPS_PER_ROTATION;

    for (int i = 0; i < STEPS_PER_ROTATION; i++)
    {
        torque_average += cogging_torque[1][i];
    }

    int friction_torque = (int)(torque_average / steps);

    for (int i = 0; i < STEPS_PER_ROTATION; i++)
    {
        cogging_torque[1][i] -= friction_torque;
    }
    return friction_torque;
}

void compensate_torque_ripples(client interface MotionControlInterface i_motion_control, client interface PositionFeedbackInterface i_position_feedback
        , client interface TorqueControlInterface i_torque_control)
{
    int position, count, status;
    int velocity_command = 10;
    int friction_constant;
    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;
    MotionControlConfig motion_ctrl_config;

    motion_ctrl_config = i_motion_control.get_motion_control_config();



    downstream_control_data.velocity_cmd = velocity_command;
    downstream_control_data.torque_cmd = 0;
    printf("set velocity %d\n", downstream_control_data.velocity_cmd);
    downstream_control_data.offset_torque = 0;

    i_motion_control.update_control_data(downstream_control_data);

    i_motion_control.enable_torque_ctrl();
    delay_milliseconds(500);
    int velocity_integral = 0;
    int counter = 0;

    float velocity_ref = 0.0;
    float velocity;
    while (1)
    {
        upstream_control_data = i_torque_control.update_upstream_control_data();
//        position = upstream_control_data.angle;
        {count, position, status} = i_position_feedback.get_position();

        motion_ctrl_config = i_motion_control.get_motion_control_config();
        downstream_control_data.offset_torque = interpolate_sensor_torque(position);
//        velocity_integral += upstream_control_data.velocity;
//        counter++;
//        if (counter >= 150)
//        {
//            velocity_ref = velocity_integral/counter;
//            counter = 0;
//            velocity_integral = 0;
//        }
//
//        if (velocity_ref >= 100 )
//        {
//            velocity = 1.0;
//            friction_constant = 58;
//        } else if (velocity_ref > 0)
//        {
//            velocity = 1.0;
//            friction_constant = 45;
//        }
//        else if (velocity_ref <= -100)
//        {
//            velocity = -1.0;
//            friction_constant = 63;
//
//        } else if (velocity_ref < 0)
//        {
//            velocity = -1.0;
//            friction_constant = 45;
//        }
//        else
//        {
//            velocity = (float)velocity_ref;
//            friction_constant = 100;
//        }
//        downstream_control_data.offset_torque += (friction_constant*velocity);
        xscope_int(POSITION, position);
        xscope_int(TORQUE_SENT, downstream_control_data.offset_torque);

        i_motion_control.update_control_data(downstream_control_data);
        delay_microseconds(500);
    }
}

void map_torque_ripples(client interface MotionControlInterface i_motion_control, client interface PositionFeedbackInterface i_position_feedback
        , client interface TorqueControlInterface i_torque_control)
{
    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;
    MotionControlConfig motion_ctrl_config;

    int velocity_command = 10;
    int number_of_turns = 0;
    int position, count, status;
    int count_start;
    int measurement_complete = 0;

    //Let time for the program to setup
    delay_milliseconds(2000);
    i_motion_control.enable_velocity_ctrl();



    downstream_control_data.offset_torque = 0;
    downstream_control_data.velocity_cmd = 0;
    motion_ctrl_config = i_motion_control.get_motion_control_config();



    downstream_control_data.velocity_cmd = velocity_command;
    i_motion_control.update_control_data(downstream_control_data);
    printf("set velocity %d\n", downstream_control_data.velocity_cmd);

    i_motion_control.update_control_data(downstream_control_data);

    delay_milliseconds(500);

    printf("Start Measurements\n");

    {count_start, position, status} = i_position_feedback.get_position();

    float resolution = SENSOR_RESOLUTION, steps = STEPS_PER_ROTATION;
    int position_step = (int)(resolution/steps);


    short counter_average [2][STEPS_PER_ROTATION] = {{0}};

    while (number_of_turns < 5 || !measurement_complete)
    {
        {count, position, status} = i_position_feedback.get_position();

        if (!(position%position_step))
        {
            upstream_control_data = i_torque_control.update_upstream_control_data();
            int index = position / position_step;
            if (index < STEPS_PER_ROTATION)
            {
                cogging_torque [0][index] = position;
                cogging_torque [1][index] += upstream_control_data.torque_set;
                counter_average [0][index] = position;
                counter_average [1][index] ++;
            }
            int next_turn = number_of_turns + 1;
            number_of_turns = (count-count_start)/SENSOR_RESOLUTION;
            if(next_turn == number_of_turns)
            {
                measurement_complete = 1;
                for (int i = 0; i < STEPS_PER_ROTATION; i++)
                {
                    if (!counter_average [1][i])
                    {
                        measurement_complete = 0;
                    }
                }
            }
        }
    }

    for (int i = 0; i < STEPS_PER_ROTATION; i++)
    {
        if (counter_average [1][i])
        {
            cogging_torque [1][i] = cogging_torque [1][i]/counter_average[1][i];
        }
    }

    remove_friction_torque();

    for (int index = 0; index< STEPS_PER_ROTATION ; index++)
    {
            printf("%d, ", cogging_torque[0][index]);
            printf("%d, ", cogging_torque[1][index]);
            printf("%d\n ", counter_average [1][index]);
    }


    downstream_control_data.velocity_cmd = 0;
    i_motion_control.enable_velocity_ctrl();
    i_motion_control.update_control_data(downstream_control_data);
}

/*
 * torque_ripple_correction.xc
 *
 *  Created on: Mar 20, 2017
 *      Author: jgofre
 */

#include <torque_ripple_correction.h>
#include <stdio.h>
#include <data_processing.h>

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
void map_torque_ripples(client interface PositionVelocityCtrlInterface i_position_control, client interface PositionFeedbackInterface i_position_feedback
        )
{
    DownstreamControlData downstream_control_data;
    UpstreamControlData upstream_control_data;

    short int torque_values [SENSOR_RESOLUTION] = {0};
    short int datas [SENSOR_RESOLUTION] = {0};
    int velocity_command = 1000;
    int number_of_turns = 0;
    int position, count, status;
    int count_start;
    downstream_control_data.offset_torque = 0;
    downstream_control_data.velocity_cmd = velocity_command;
    i_position_control.enable_velocity_ctrl();
    i_position_control.update_control_data(downstream_control_data);

    delay_milliseconds(500);
    {count_start, position, status} = i_position_feedback.get_position();
    while (1)
    {
//        {count, position, status} = i_position_feedback.get_position();
//        //get torque
//        //        printf("Count : %d\n", count);
//
////        upstream_control_data = i_motorcontrol.update_upstream_control_data();
//        torque_values[position] += upstream_control_data.computed_torque;
//        datas [position] ++;
//        number_of_turns = (count-count_start)/SENSOR_RESOLUTION;
    }
//    interpolate_sensor_data(torque_values, datas, SENSOR_RESOLUTION);
//
//
//    for (int i = 0; i< SENSOR_RESOLUTION ; i++)
//    {
//            printf ("\n%d", torque_values[i]);
//
//    }
//    printf ("\n");
//    downstream_control_data.velocity_cmd = 0;
//    i_position_control.enable_velocity_ctrl();
//    i_position_control.update_control_data(downstream_control_data);
//    exit(1);
}

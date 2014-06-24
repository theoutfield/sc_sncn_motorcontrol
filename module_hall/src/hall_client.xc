/**
 * \file hall_client.xc
 * \brief Hall Sensor Client Functions
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
*/

#include <hall_config.h>

//TODO remove these dependencies
#include <bldc_motor_config.h>

int get_hall_position(chanend c_hall)
{
    int position;
    c_hall <: HALL_POS_REQ;
    c_hall :> position;
    return position;
}

{int , int} get_hall_position_absolute(chanend c_hall)
{
    int position;
    int direction;
    c_hall <: HALL_ABSOLUTE_POS_REQ;
    c_hall :> position;
    c_hall :> direction;
    return {position, direction};
}

int get_hall_velocity(chanend c_hall, hall_par &hall_params)
{
    int velocity;
    c_hall <: HALL_VELOCITY_REQ;
    c_hall :> velocity;
    velocity = ((velocity/FILTER_LENGTH_HALL) * 1000 * 60) / (hall_params.max_ticks_per_turn);
    return velocity;
}

void reset_hall_count(chanend c_hall, int offset)
{
    c_hall <: RESET_HALL_COUNT;
    c_hall <: offset;
}

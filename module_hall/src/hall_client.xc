/**
 * @file hall_client.xc
 * @author Synapticon GmbH <support@synapticon.com>
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

int get_hall_velocity(chanend c_hall)
{
    int velocity;
    int max_ticks_per_turn;

    //Get velocity
    c_hall <: HALL_VELOCITY_REQ;
    c_hall :> velocity;

    //Get velocity filter parameters FIXME: filtering schould be done in server
    c_hall <: HALL_FILTER_PARAM_REQ;
    c_hall :> max_ticks_per_turn;

    //Apply filter to velocity FIXME: filtering schould be done in server
    velocity = ((velocity/FILTER_LENGTH_HALL) * 1000 * 60) / (max_ticks_per_turn);

    return velocity;
}

void reset_hall_count(chanend c_hall, int offset)
{
    c_hall <: HALL_RESET_COUNT_REQ;
    c_hall <: offset;
}

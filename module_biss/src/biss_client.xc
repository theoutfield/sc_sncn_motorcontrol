/**
 * @file biss_client.xc
 * @brief BiSS Encoder Client Functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_client.h>
#include <filter_blocks.h>

void init_biss_velocity_params(biss_velocity_par &biss_velocity_params)
{
    biss_velocity_params.previous_position = 0;
    biss_velocity_params.old_difference = 0;
    biss_velocity_params.filter_length = 8;
    biss_velocity_params.index = 0;
    init_filter(biss_velocity_params.filter_buffer,
                biss_velocity_params.index,
                biss_velocity_params.filter_length);
    return;
}

unsigned int get_biss_position(client interface i_biss i_biss) {
    unsigned int position;
    { void, position, void } = i_biss.get_position();
    return position;
}

int get_biss_position_absolute(client interface i_biss i_biss) {
    int count;
    { count, void, void } = i_biss.get_position();
    return count;
}

{ int, unsigned int, unsigned int } get_biss_state(client interface i_biss i_biss) {
    int count;
    unsigned int position, status;
    { count, position, status } = i_biss.get_position();
    return { count, position, status };
}

unsigned int get_biss_angle_electrica(client interface i_biss i_biss) {
    return i_biss.get_angle_electrical();
}


void set_biss_params(client interface i_biss i_biss, biss_par biss_params) {
    i_biss.set_params(biss_params);
}

int get_biss_velocity(client interface i_biss i_biss, biss_par & biss_params, biss_velocity_par &biss_velocity_params, int count)
{
    int biss_crossover = (1 << biss_params.singleturn_resolution) - (1 << biss_params.singleturn_resolution)/10;
    if (count == 0)
        count = get_biss_position_absolute(i_biss);
    int difference = count - biss_velocity_params.previous_position;
    if(difference > biss_crossover || difference < -biss_crossover)
        difference = biss_velocity_params.old_difference;
    biss_velocity_params.previous_position = count;
    biss_velocity_params.old_difference = difference;
    return (filter(biss_velocity_params.filter_buffer, biss_velocity_params.index, \
                   biss_velocity_params.filter_length, difference)*1000*60) / (1 << biss_params.singleturn_resolution);
}

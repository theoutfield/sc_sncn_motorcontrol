/**
 * @file biss_client.xc
 * @brief BiSS Encoder Client Functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_client.h>

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

/**
 * \file misc.xc
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <refclk.h>
#include <qei_client.h>
#include <hall_client.h>
#include <comm_loop_client.h>
#include <drive_config.h>
#include <qei_config.h>

int detect_sensor_placement(chanend c_hall, chanend c_qei, chanend c_commutation)
{
    int times = 50;
    int valid;
    int current_pos_q;
    int current_pos_h;
    int hall_di;
    int avg_q = 0;
    int sum_h = 0;
    int difference_h;
    int difference_q;
    int previous_position_q = 0;
    int previous_position_h = 0;
    timer t;
    unsigned int time;
    int i;
    int sensor_placement_type; //1 in phase -1 out of phase
    int init_state;

    while(1) {
        init_state = __check_commutation_init(c_commutation);
        if (init_state == INIT) {
            printstrln("commutation intialized");
            init_state = INIT_BUSY;
            break;
        }
    }

    set_commutation_sinusoidal(c_commutation, 400);
    t :> time;

    while (sum_h ==0) {
        for (i=0; i<times ; i++) {
            t when timerafter(time+100000) :> time;
            { current_pos_h, hall_di } = get_hall_position_absolute(c_hall);
            difference_h = current_pos_h - previous_position_h;
            sum_h = sum_h + difference_h;
            previous_position_h = current_pos_h;
        }
    }

    t :> time;
    while (avg_q == 0) {
        for (i = 0; i<times ; i++) {
            t when timerafter(time+3000) :> time;
            { current_pos_q, valid } = get_qei_position_absolute(c_qei);
            difference_q = current_pos_q - previous_position_q;
            if (difference_h > 10) {
                difference_h = 0;
            } else if (difference_h < -10) {
                difference_h = 0;
            }
            avg_q = avg_q + difference_q;
            previous_position_q = current_pos_q;
        }
    }
    //avg_q = 0;
    printintln(sum_h);
    printintln(avg_q);
    if ( (avg_q > 0) && (sum_h < 0 ) ) {
        sensor_placement_type = INVERTED;
        printstrln("inverted case1");
    }

    if ( (avg_q < 0) && (sum_h > 0) ) {
        sensor_placement_type = INVERTED;
        printstrln("inverted case2");
    }

    if ( (avg_q > 0) && (sum_h > 0) ) {
        sensor_placement_type = NORMAL;
        printstrln("normal case1");
    }

    if ( (avg_q < 0) && (sum_h < 0) ) {
        sensor_placement_type = NORMAL;
        printstrln("normal case2");
    }

    set_commutation_sinusoidal(c_commutation ,0);
    return sensor_placement_type;
}


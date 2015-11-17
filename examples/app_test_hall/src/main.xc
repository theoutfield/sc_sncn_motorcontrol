/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <print.h>
#include <hall_client.h>
#include <hall_server.h>
#include <xscope.h>

#define ENABLE_XSCOPE

port p_hall = HALL_PORT;
#ifdef DC1K
port p_ifm_encoder_hall_select_ext_d4to5 = SELECTION_HALL_ENCODER_PORT;
#endif

/* Test Hall Sensor Client */
void hall_test(chanend c_hall)
{
    int position = 0;
    int velocity = 0;
    int count = 0;
    int direction;
    int old_count = 0;
    int pins = 0;

    while(1)
    {
        /* get position from Hall Sensor */
        {count, direction} = get_hall_position_absolute(c_hall);
        position = get_hall_position(c_hall);

        /* get velocity from Hall Sensor */
        velocity = get_hall_velocity(c_hall);

        /* get pins state from Hall Sensor */
        pins = get_hall_pinstate(c_hall);

#ifndef ENABLE_XSCOPE
        if (count != old_count) {
            printstr("Count: ");
            printint(count);
            printstr(" ");
            printstr("Position: ");
            printint(position);
            printstr(" ");
            printstr("Velocity: ");
            printint(velocity);
            printstr(" ");
            printstr("Pins: ");
            printchar('0'+((pins&0b100)>>2));
            printchar('0'+((pins&0b10)>>1));
            printcharln('0'+(pins&1));
        }
        old_count = count;
#else
        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);
        xscope_int(A, (pins&1)*1000); // scale to 1000 for easier display in xscope
        xscope_int(B, (pins&0b10)*500);
        xscope_int(C, (pins&0b100)*250);
#endif
    }
}



int main(void)
{
    chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;  // hall channels

    par
    {
        on tile[APP_TILE]:
        {
            /* Test Hall Sensor Client */
            par
            {
                hall_test(c_hall_p1);
            }
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            /* Hall Server */
            {
                hall_par hall_params;
#ifdef DC1K
                //connector 1
                p_ifm_encoder_hall_select_ext_d4to5 <: SET_ALL_AS_HALL;
                run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6,
                                        p_hall, hall_params); // channel priority 1,2..6

#else
                run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6,
                        p_hall, hall_params); // channel priority 1,2..6
#endif
            }

        }

    }

    return 0;
}

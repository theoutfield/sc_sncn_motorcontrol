/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <print.h>
#include <hall_server.h>
#include <hall_server.h>
#include <xscope.h>

#define ENABLE_XSCOPE

HallPorts hall_ports = HALL_PORTS;

/* Test Hall Sensor Client */
void hall_test(interface HallInterface client i_hall)
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
        {count, direction} = i_hall.get_hall_position_absolute();
        position = i_hall.get_hall_position();

        /* get velocity from Hall Sensor */
        velocity = i_hall.get_hall_velocity();

        /* get pins state from Hall Sensor */
        pins = i_hall.get_hall_pinstate();

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
    interface HallInterface i_hall[5];

    par
    {
        on tile[APP_TILE]: hall_test(i_hall[0]);
        on tile[IFM_TILE]: run_hall(i_hall, hall_ports); // channel priority 1,2..6

    }

    return 0;
}

/*
 * app_test_lan9252.xc
 *
 *  Created on: Sep 8, 2016
 *      Author: Frank Jeschke <fjeschke@synapticon.com>
 *
 *  Copyright, 2016 Synapticon GmbH
 */
#include <CORE_C22-rev-a.bsp>

//#include <stdint.h>
//#include <inttypes.h>
//#include <time.h> /* has to be here because clock type gives a clash with the type in xs1.h */

#include "as5050a.h"
#include <spi_master.h>

SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
//unsigned int position = 0;
in port INTPIN = on tile[IFM_TILE]: XS1_PORT_1A;
int x;
inline static void delay_ten_nanosecond(timer t, int delay)
{
    int ts;
    t:>ts;
    t when timerafter(ts+delay) :> void;
}

void as5050a_runtest(SPIPorts &spi_ports)
{
    unsigned int position = 0;
    initspi_ports(spi_ports);
   // delay_milliseconds(2);

    delay_microseconds(600);

            while (1)
    {
    /* check for INT/ is low to continue */
    INTPIN :> x;
    //printstr("INT/ = "); printintln(x);
    if (x == 0)
    {
    position = readAngleValue(spi_ports);
    printstr("position = "); printintln(position);
    }
    else
        printstrln("sensor not ready for next command");
    }
}

int main(void)
{

    par {
        on tile[IFM_TILE]: {

            as5050a_runtest(spi_ports);

        }
    }

    return 0;
}

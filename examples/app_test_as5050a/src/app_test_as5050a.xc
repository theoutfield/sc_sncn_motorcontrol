#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

//#include <stdint.h>
//#include <inttypes.h>
//#include <time.h> /* has to be here because clock type gives a clash with the type in xs1.h */

#include <as5050a.h>
//#include <spi_master.h>
#include <position_feedback_service.h>

SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
//unsigned int position = 0;

//in port INTPIN = on tile[IFM_TILE]: XS1_PORT_1A;
//int check_pin1;
inline static void delay_ten_nanosecond(timer t, int delay)
{
    int ts;
    t:>ts;
    t when timerafter(ts+delay) :> void;
}

void as5050a_runtest(SPIPorts &spi_ports)
{
    int start, finish, time;
    unsigned int position = 0;
    initas5050a(spi_ports);
    timer t;

            while (1)
    {
//    /* check for INT/ is low to continue */
//    INTPIN :> check_pin1;
//    //printstr("INT/ = "); printintln(x);
//    if (check_pin1 == 0)
//    {
//        t :> start;
//
//        position = readAngleValue(spi_ports);
////        t :> finish;
////        time = finish - start;
//        printstr("position = "); printintln(position); //printstr(" reading time = "); printintln(time);
//
//    }
//    else
//        printstrln("sensor not ready for next command");
    position = readAngleValue(spi_ports);
    printstr("position = "); printintln(position); //printstr(" reading time = "); printintln(time);
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

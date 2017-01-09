#include <CORE_C22-rev-a.bsp>

//#include <stdint.h>
//#include <inttypes.h>
//#include <time.h> /* has to be here because clock type gives a clash with the type in xs1.h */

in port P8C = on tile[IFM_TILE]: XS1_PORT_8C;

//int a = 0,b = 0,c = 0,d = 0,e = 0,f = 0,g = 0,h = 0;
int data = 0;
inline static void delay_ten_nanosecond(timer t, int delay)
{
    int ts;
    t:>ts;
    t when timerafter(ts+delay) :> void;
}

int main(void)
{
    par {
        on tile[IFM_TILE]: {

            delay_milliseconds(2);

            while (1)
            {

                P8C :> data;
                printstr("Position = "); printintln(data);
            }

        }
    }

    return 0;
}

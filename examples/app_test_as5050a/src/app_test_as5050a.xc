#include <CORE_C22-rev-a.bsp>
//#include <IFM_DC100-rev-b.bsp>
#include <position_feedback_service.h>

SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;

void as5050a_runtest(SPIPorts &spi_ports)
{
    //int start, finish, time;
    unsigned int position = 0;
    initas5050a(spi_ports);
    //timer t;

            while (1)
    {
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

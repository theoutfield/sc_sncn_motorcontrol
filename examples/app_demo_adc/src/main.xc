/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

#include <adc_service.h>
#include <motor_control_interfaces.h>
#include <demo_adc.h>

ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

int main(void)
{
    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];

    par
    {
        on tile[APP_TILE]:
        {
            demo_adc(i_adc[1]);
        }

        on tile[IFM_TILE]:
        {
            adc_service(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, STD_MOTOR_CTRL_MODE);
        }
    }

    return 0;
}

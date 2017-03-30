/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

#include <demo_adc.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>
#include <motor_control_interfaces.h>

ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

int main(void)
{
    // ADC interface
    interface ADCInterface i_adc[2];

    par
    {
        on tile[APP_TILE]:
        {
            adc_client_demo(i_adc[1], AD_7949);
        }

        on tile[IFM_TILE]:
        {
            if(!isnull(adc_ports.ad7949_ports.clk))         adc_ad7949_service_demo(adc_ports.ad7949_ports, i_adc);
            else if(!isnull(adc_ports.ad7265_ports.xclk))   adc_ad7265_service_demo(adc_ports.ad7265_ports, i_adc);
        }
    }

    return 0;
}

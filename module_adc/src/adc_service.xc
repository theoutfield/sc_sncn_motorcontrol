#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

void adc_service(
        ADCPorts &adc_ports,
        interface ADCInterface server i_adc[2],
        interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec, int operational_mode)
{
    if(!isnull(adc_ports.ad7949_ports.clk))
        adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, i_watchdog, operational_mode);
    else if(!isnull(adc_ports.ad7265_ports.xclk))
        adc_ad7265(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog, operational_mode);
}

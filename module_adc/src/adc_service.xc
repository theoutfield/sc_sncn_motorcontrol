#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

/**
 * @brief Service providing readings from the ADC chip in your SOMANET device.
 * Measurements can be sampled on requests through i_adc interfaces.
 *
 * @param adc_ports             Ports structure defining where to access the ADC chip signals.
 * @param i_adc[2]              Array of communication interfaces to handle up to 2 different clients.
 * @param i_watchdog            Interface to communicate with watchdog service
 * @param tile_usec             Reference clock frequency of IF2 tile (in MHz)
 * @param operational_mode      Integer type to select between SINGLE_ENDED/FULLY_DIFFERENTIAL modes
 *
 * @return void
 */
void adc_service(
        ADCPorts &adc_ports,
        interface ADCInterface server i_adc[2],
        interface WatchdogInterface client ?i_watchdog, int tile_usec, int operational_mode)
{
    if(!isnull(adc_ports.ad7949_ports.clk))
        adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, i_watchdog, operational_mode, tile_usec);
    else if(!isnull(adc_ports.ad7265_ports.xclk))
        adc_ad7265(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog, operational_mode, tile_usec);
}// adc_service

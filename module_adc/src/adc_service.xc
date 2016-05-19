#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

void adc_service(ADCPorts &adc_ports, chanend ?c_trigger, interface ADCInterface server i_adc[2], interface WatchdogInterface client ?i_watchdog){

    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    printstr(">>   SOMANET ADC SERVICE STARTING...\n");

    if(isnull(c_trigger)){ // Check for triggered sampling channel

        if(!isnull(adc_ports.ad7949_ports.clk)){ // Check which ADC is configured

            adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, i_watchdog);

        } else if(!isnull(adc_ports.ad7265_ports.xclk)){

            if(ADC_FIXED_CHANNEL_OPERATION)
                adc_ad7256_fixed_channel(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog);
            else
                adc_ad7256(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog);

        } else {

            printstr("adc_service: ERROR No ADC configured");

        }
    } else{

        if(!isnull(adc_ports.ad7949_ports.clk)){  // Check which ADC is configured

            adc_ad7949_triggered(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, c_trigger, i_watchdog);

        } else if(!isnull(adc_ports.ad7265_ports.xclk)){

            adc_ad7256_triggered(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, c_trigger, i_watchdog);

        } else {

            printstr("adc_service: ERROR No ADC configured");

        }
    }
}

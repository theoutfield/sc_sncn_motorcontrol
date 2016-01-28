#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

void adc_service(ADCPorts &adc_ports, chanend ?c_trigger, interface ADCInterface server i_adc[2]){

    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    printstr(">>   SOMANET ADC SERVICE STARTING...\n");

    if(isnull(c_trigger)){

        // There is not triggered sampling
        if(!isnull(adc_ports.ad7949_ports.clk)){

            adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config);
        }else{

            adc_ad7256(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config);
        }
    }else{

        // There is triggering
        if(!isnull(adc_ports.ad7949_ports.clk)){

            adc_ad7949_triggered(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, c_trigger);
        }else{

            adc_ad7256_triggered(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, c_trigger);
        }
    }
}

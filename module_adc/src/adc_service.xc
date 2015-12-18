#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

void adc_service(ADCPorts &adc_ports, chanend ?c_trigger, interface ADCInterface server adc_interface[3]){

    if(isnull(c_trigger)){
        // There is not triggering by PWM
        chan c_dummy;

        if(!isnull(adc_ports.ad7949_ports.clk)){

            adc_ad7949(adc_interface, adc_ports.ad7949_ports, adc_ports.current_sensor_config, c_dummy);
        }else{

            adc_ad7256(adc_interface, adc_ports.ad7265_ports, adc_ports.current_sensor_config, c_dummy);
        }
    }else{

        // There is triggering
        if(!isnull(adc_ports.ad7949_ports.clk)){

            adc_ad7949(adc_interface, adc_ports.ad7949_ports, adc_ports.current_sensor_config, c_trigger);
        }else{

            adc_ad7256(adc_interface, adc_ports.ad7265_ports, adc_ports.current_sensor_config, c_trigger);
        }
    }
}

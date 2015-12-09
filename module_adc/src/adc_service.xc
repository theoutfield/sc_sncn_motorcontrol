#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_server_ad7949.h>

void adc_service(interface ADCInterface server adc_interface[3], ADCPorts &adc_ports, chanend ?c_trigger){

    if(!isnull(adc_ports.ad7949_ports.clk)){

        if(isnull(c_trigger)){

            chan c_dummy;
            adc_ad7949_triggered(adc_interface, adc_ports.ad7949_ports, c_dummy);

        }else{

            adc_ad7949_triggered(adc_interface, adc_ports.ad7949_ports, c_trigger);
        }
    }

    if(!isnull(adc_ports.ad7265_ports.xclk)){

    }

}

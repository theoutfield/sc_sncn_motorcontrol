#include <xs1.h>
#include <adc.h>
#include <adc_7265.h>
#include <adc_server_ad7949.h>

void run_adc_service(interface ADCInterface server adc_interface, ADCPorts &adc_ports, chanend ?c_trigger){

    if(!isnull(adc_ports.ad7949_ports.clk)){
        adc_ad7949_triggered(adc_interface, adc_ports.ad7949_ports, c_trigger);
    }

    if(!isnull(adc_ports.ad7265_ports.xclk)){

    }

}

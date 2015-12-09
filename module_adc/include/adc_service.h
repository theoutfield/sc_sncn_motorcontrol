/**
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 **/

#pragma once

#include <xs1.h>

interface ADCInterface{
   // void calibrate();
    {int, int, int, int, int, int, int, int} get_all();
    {int, int} get_currents();
    {int, int} get_external_inputs();
};

typedef struct {
     buffered out port:32 ?sclk_conv_mosib_mosia;
     in buffered port:32 ?data_a;
     in buffered port:32 ?data_b;
     clock ?clk;
}AD7949Ports;

typedef struct {
    in buffered port:32 ?p32_data[2]; // Array of 32-bit buffered ADC data ports
    clock ?xclk; // Internal XMOS clock
    out port ?p1_serial_clk; // 1-bit port connecting to external ADC serial clock
    port ?p1_ready;   // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
    out port ?p4_mux; // 4-bit port used to control multiplexor on ADC chip
} AD7265Ports;

typedef struct {
    AD7949Ports ad7949_ports;
    AD7265Ports ad7265_ports;
} ADCPorts;

void adc_service(interface ADCInterface server adc_interface[3], ADCPorts &adc_ports, chanend ?c_trigger);

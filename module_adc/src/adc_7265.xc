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

#include "adc_7265.h"

#define SHIFTING_BITS   1

void adc_ad7265_singleshot(AD7265Ports &adc_ports, int adc_data[2][6],
                            unsigned char config,
                            unsigned char &port_id,
                            unsigned int stabilizing_ticks){

    unsigned inp_val = 0, tmp_val = 0;
    unsigned time_stamp; // Time stamp
    unsigned char mux_config;
    timer t;
    unsigned int ts;

///////////////First we sample currents///////////////////
    mux_config = (1 << 3); //Channel 0 - Config 1 : Currents
    adc_ports.p4_mux <: mux_config;
    clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
    clearbuf( adc_ports.p32_data[1] );

    t :> ts;
    t when timerafter(ts + stabilizing_ticks) :> ts;

    adc_ports.p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
    time_stamp += (ADC_TOTAL_BITS+2); // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
    adc_ports.p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)

    sync( adc_ports.p1_ready ); // Wait until port has completed any pending outputs

    // Get data from port a
    endin( adc_ports.p32_data[0] ); // End the previous input on this buffered port
    adc_ports.p32_data[0] :> inp_val; // Get new input
    tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
    tmp_val = tmp_val >> (SHIFTING_BITS+1);
    tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
    adc_data[0][0] = (int)tmp_val;
    adc_data[0][0] = adc_data[0][0] << 2;  // So we extend to 14Bit: 0 - 16384

    // Get data from port b
    endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
    adc_ports.p32_data[1] :> inp_val; // Get new input
    tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
    tmp_val = tmp_val >> (SHIFTING_BITS+1);
    tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
    adc_data[1][0] = (int)tmp_val;
    adc_data[1][0] = adc_data[1][0] << 2;  // So we extend to 14Bit: 0 - 16384


    if(port_id == 0)
        return;

/////////////Then we sample the requested index///////////////////////
    mux_config = (config << 3) | (port_id - 1);
    adc_ports.p4_mux <: mux_config;
    clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
    clearbuf( adc_ports.p32_data[1] );

    t :> ts;
    t when timerafter(ts + stabilizing_ticks) :> ts;

    adc_ports.p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
    time_stamp += (ADC_TOTAL_BITS+2); // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
    adc_ports.p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)

    sync( adc_ports.p1_ready ); // Wait until port has completed any pending outputs

    // Get data from port a
    endin( adc_ports.p32_data[0] ); // End the previous input on this buffered port
    adc_ports.p32_data[0] :> inp_val; // Get new input
    tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
    if(config == 0) tmp_val = tmp_val >> SHIFTING_BITS;
    else tmp_val = tmp_val >> (SHIFTING_BITS+1);
    tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
    adc_data[0][port_id - 1] = (int)tmp_val;
    adc_data[0][port_id - 1] = adc_data[0][port_id - 1] << 2; // So we extend to 14Bit: 0 - 16384

    // Get data from port b
    endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
    adc_ports.p32_data[1] :> inp_val; // Get new input
    tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
    if(config == 0) tmp_val = tmp_val >> SHIFTING_BITS;
    else tmp_val = tmp_val >> (SHIFTING_BITS+1);
    tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
    adc_data[1][port_id - 1] = (int)tmp_val;
    adc_data[1][port_id - 1] = adc_data[1][port_id - 1] << 2; // So we extend to 14Bit: 0 - 16384

}

/*****************************************************************************/
void configure_adc_ports_7265( // Configure all ADC data ports
    in buffered port:32 ?p32_data_a, // Array of 32-bit buffered ADC data ports
    in buffered port:32 ?p32_data_b,
    clock ?xclk, // XMOS internal clock
    out port ?p1_serial_clk, // 1-bit Port connecting to external ADC serial clock
    port p1_ready,   // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
    out port p4_mux // 4-bit port used to control multiplexor on ADC chip
)
{

    /* xclk & p1_ready, are used as the clock & ready signals respectively for controlling the following 2 functions:-
        (1) Reading the Digital data from the AD7265 into an XMOS buffered 1-bit port
        (2) Initiating an Analogue-to-digital conversion on the AD7265 chip.

        For (1), Referring to XMOS XS1 library documentation ...
        By default, the ports are read on the rising edge of the clock, and when the ready signal is high.

        For (2), Referring to the  AD7265 data-sheet ...
        p1_ready is used to control CSi (Chip Select Inverted)
        When signal CSi falls, ( 1 -> 0 ) A2D conversion starts. When CSi rises ( 0 -> 1 ), conversion halts.
    The digital outputs are tri-state when CSi is high (1).
        xclk is used to control SCLK (Serial Clock).
        Succesive bits of the digital sample are output after a falling edge of SCLK. In the following order ...
        [0, 0, Bit_11, Bit_10, ... Bit_1, Bit_0, 0, 0]. If CSi rises early, the LSB bits (and zeros) are NOT output.

        We require the analogue signal to be sampled on the falling edge of the clock,
        According to the AD7265 data-sheet, the output data is ready to be sampled 36 ns after the falling edge.
        If we use the rising edge of the xclk to read the data, an xclk frequency of 13.8 MHz or less is required.
        Frequencies above 13.9 MHz require the data to be read on the next falling edge of xclk.

        We require the analogue signal to be sampled when CSi goes low,
        and we require data to be read when the ready signal goes high.
        By using the set_port_inv() function to invert the ready signal, it can be used for both (1) & (2).
        NB If an inverted port is used as ready signals to control another port,
    the internal signal (used by XMOS port) is inverted with respect to the external signal (used to control AD7265).
    */

    // configure the clock to be (at most) 13 MHz. (NB This is independent of the XCore Reference Frequency)
    configure_clock_rate_at_most( xclk ,ADC_SCLK_MHZ ,1 ); // XS1 library call
    configure_port_clock_output( p1_serial_clk, xclk ); // Drive ADC serial clock port with XMOS clock

    configure_out_port( p1_ready ,xclk ,0 ); // Set initial value of port to 0 ( NOT ready )
    set_port_inv( p1_ready ); // Invert p1_ready for connection to AD7265, which has active low

    // For this port, configure to read into buffer when using the serial clock
    configure_in_port_strobed_slave( p32_data_a ,p1_ready ,xclk ); // XS1 Library call
    configure_in_port_strobed_slave( p32_data_b ,p1_ready ,xclk ); // XS1 Library call


    start_clock( xclk );    // Start the ADC serial clock port

} // configure_adc_ports_7265

void adc_ad7256(interface ADCInterface server iADC[2], AD7265Ports &adc_ports, CurrentSensorsConfig &current_sensor_config)
{
    int adc_data[2][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0}};
    unsigned char sampling_port = 0;

    //Calibration variables
    int i_calib_a = 0, i_calib_b = 0, i = 0, Icalibrated_a = 0, Icalibrated_b = 0;

    configure_adc_ports_7265( adc_ports.p32_data[0], adc_ports.p32_data[1], adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    //Calibration
    while (i < ADC_CALIB_POINTS) {
        // get ADC reading

        adc_ad7265_singleshot(adc_ports, adc_data, 1, sampling_port, 0);

        if (adc_data[0][0]>0 && adc_data[0][0]<16382  &&  adc_data[1][0]>0 && adc_data[1][0]<16382) {
            i_calib_a += adc_data[0][0];
            i_calib_b += adc_data[1][0];
            i++;
            if (i == ADC_CALIB_POINTS) {
                break;
            }
        }
    }

   i_calib_a = (i_calib_a >> Factor);
   i_calib_b = (i_calib_b >> Factor);

    while(1){

    #pragma ordered
        select {
        case iADC[int i].get_currents() -> {int adc_A, int adc_B}:

                // Config: 1 Port: 1
                sampling_port = 0;  // If provided port is 0, just once sampling takes place: current,
                                    // and no additional sampling is done
                adc_ad7265_singleshot(adc_ports, adc_data, 1, sampling_port, 200);
                Icalibrated_a = ((int) adc_data[0][0]) - i_calib_a;
                Icalibrated_b =((int) adc_data[1][0]) - i_calib_b;

                adc_A = current_sensor_config.sign_phase_b * Icalibrated_a;
                adc_B = current_sensor_config.sign_phase_c * Icalibrated_b;

                break;

        case iADC[int i].get_temperature() -> {int out_temp}:

                sampling_port = 5;
                adc_ad7265_singleshot(adc_ports, adc_data, 1, sampling_port, 200);

                out_temp = adc_data[0][4];

             break;

        case iADC[int i].get_external_inputs() -> {int ext_a, int ext_b}:

                //We sample the external inputs as differential inputs: ch3 - ch4
                // Config: 0 Port: 3
                sampling_port = 3;
                adc_ad7265_singleshot(adc_ports, adc_data, 0, sampling_port, 200);

                ext_a = adc_data[0][2];
                ext_b = adc_data[1][2];

                break;

        case iADC[int i].helper_amps_to_ticks(float amps) -> int out_ticks:


                if(amps >= current_sensor_config.current_sensor_amplitude)
                     out_ticks = MAX_ADC_VALUE/2; break;
                if(amps <= -current_sensor_config.current_sensor_amplitude)
                    out_ticks = -MAX_ADC_VALUE/2; break;

                out_ticks = (int) amps * (MAX_ADC_VALUE/(2*current_sensor_config.current_sensor_amplitude));

                break;

        case iADC[int i].helper_ticks_to_amps(int ticks) -> float out_amps:

                if(ticks >= MAX_ADC_VALUE/2)
                    out_amps = current_sensor_config.current_sensor_amplitude; break;
                if(ticks <= -MAX_ADC_VALUE/2)
                    out_amps = -current_sensor_config.current_sensor_amplitude; break;

                out_amps = ticks/(MAX_ADC_VALUE/2.0) * current_sensor_config.current_sensor_amplitude;

                break;

        }//eof select
    }//eof while
}

void adc_ad7256_triggered(interface ADCInterface server iADC[2], AD7265Ports &adc_ports, CurrentSensorsConfig &current_sensor_config, chanend c_trig)
{
    timer t;
    unsigned int ts;
    unsigned char ct;
    int adc_data[2][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0}};
    unsigned char sampling_port = 0;

    //Calibration variables
    int i_calib_a = 0, i_calib_b = 0, i = 0, Icalibrated_a = 0, Icalibrated_b = 0;

    configure_adc_ports_7265( adc_ports.p32_data[0], adc_ports.p32_data[1], adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    //Calibration
    while (i < ADC_CALIB_POINTS) {
        // get ADC reading

        adc_ad7265_singleshot(adc_ports, adc_data, 1, sampling_port, 0);

        if (adc_data[0][0]>0 && adc_data[0][0]<16382  &&  adc_data[1][0]>0 && adc_data[1][0]<16382) {
            i_calib_a += adc_data[0][0];
            i_calib_b += adc_data[1][0];
            i++;
            if (i == ADC_CALIB_POINTS) {
                break;
            }
        }
    }

   i_calib_a = (i_calib_a >> Factor);
   i_calib_b = (i_calib_b >> Factor);

    while(1){

    #pragma ordered
        select {
        case inct_byref(c_trig, ct):
                if (ct == XS1_CT_END)
                {
                    t :> ts;

                    sampling_port++;
                    if(sampling_port > 6){
                        sampling_port = 2;
                    }

                    t when timerafter(ts + 7080) :> ts; // 6200
                    //Sampling
                    adc_ad7265_singleshot(adc_ports, adc_data, 1, sampling_port, 200);
                }

                break;

        case iADC[int i].get_currents() -> {int adc_A, int adc_B}:

                adc_A = current_sensor_config.sign_phase_b * Icalibrated_a;
                adc_B = current_sensor_config.sign_phase_c * Icalibrated_b;

                break;

        case iADC[int i].get_temperature() -> {int out_temp}:

                out_temp = adc_data[0][4];

             break;

        case iADC[int i].get_external_inputs() -> {int ext_a, int ext_b}:

                //We sample the external inputs as differential inputs: ch3 - ch4
                // Config: 0 Port: 3
                sampling_port = 3;
                adc_ad7265_singleshot(adc_ports, adc_data, 0, sampling_port, 200);

                ext_a = adc_data[0][2];
                ext_b = adc_data[1][2];

                break;

        case iADC[int i].helper_amps_to_ticks(float amps) -> int out_ticks:


                if(amps >= current_sensor_config.current_sensor_amplitude)
                     out_ticks = MAX_ADC_VALUE/2; break;
                if(amps <= -current_sensor_config.current_sensor_amplitude)
                    out_ticks = -MAX_ADC_VALUE/2; break;

                out_ticks = (int) amps * (MAX_ADC_VALUE/(2*current_sensor_config.current_sensor_amplitude));

                break;

        case iADC[int i].helper_ticks_to_amps(int ticks) -> float out_amps:

                if(ticks >= MAX_ADC_VALUE/2)
                    out_amps = current_sensor_config.current_sensor_amplitude; break;
                if(ticks <= -MAX_ADC_VALUE/2)
                    out_amps = -current_sensor_config.current_sensor_amplitude; break;

                out_amps = ticks/(MAX_ADC_VALUE/2.0) * current_sensor_config.current_sensor_amplitude;

                break;

        }//eof select

        Icalibrated_a = (((int) adc_data[0][0]) - i_calib_a);
        Icalibrated_b = (((int) adc_data[1][0]) - i_calib_b);  // So we extend to 14Bit: 0 - 16384

    }//eof while
}
/*****************************************************************************/
// adc_7265.xc

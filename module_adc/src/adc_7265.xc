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

/*****************************************************************************/
void configure_adc_ports_7265( // Configure all ADC data ports
    in buffered port:32 p32_data[2], // Array of 32-bit buffered ADC data ports
    clock xclk, // XMOS internal clock
    out port p1_serial_clk, // 1-bit Port connecting to external ADC serial clock
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
    configure_in_port_strobed_slave( p32_data[0] ,p1_ready ,xclk ); // XS1 Library call
    configure_in_port_strobed_slave( p32_data[1] ,p1_ready ,xclk ); // XS1 Library call


    start_clock( xclk );    // Start the ADC serial clock port

} // configure_adc_ports_7265

/*****************************************************************************/
void foc_adc_7265_continuous_loop( // Get ADC data from AD7265 chip and send to client
    server interface ADC iADC,
    adc_ports_t &adc_ports
)
{
    // Mapping array from 'trigger channel' to 'analogue ADC mux input' See. AD7265 data-sheet

    unsigned time_stamp; // Time stamp
    int do_loop = 1;   // Flag set until loop-end condition found
    unsigned inp_val = 0, tmp_val = 0;
    int out_a = 0, out_b= 0;
    timer t;
    unsigned tin,tout;
    char config_ = 0;

    printstrln("                                           ADC Server Starts");

    //MB~ The next line gives priority to this thread, this is historical, and probably not required any more
    //MB~ However, removing it will alter timing, and a re-tuning exercise may be required!-(
    set_thread_fast_mode_on();

    configure_adc_ports_7265( adc_ports.p32_data , adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready , adc_ports.p4_mux ); // Configure all ADC data ports
    unsigned char mux_config = AD7265_MUX_DEFAULT_CONFIG;

    // Loop until termination condition detected
    while (do_loop) //
    {
        t :> tin;

        //delay_milliseconds(10);

        //Sampling time < 3us
        adc_ports.p4_mux <: mux_config; // Signal to Multiplexor which input to use for this trigger

        clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
        clearbuf( adc_ports.p32_data[1] );

        adc_ports.p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
        time_stamp += (ADC_TOTAL_BITS+2); // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
        adc_ports.p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)

        sync( adc_ports.p1_ready ); // Wait until port has completed any pending outputs

        // Get data from port a
        endin( adc_ports.p32_data[0] ); // End the previous input on this buffered port
        adc_ports.p32_data[0] :> inp_val; // Get new input
        tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
      //  printhexln(tmp_val);
        if(config_ == 0) tmp_val = tmp_val >> SHIFTING_BITS;
        else tmp_val = tmp_val >> (SHIFTING_BITS+1);
        tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
        out_a = (int)tmp_val;

        // Get data from port b
        endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
        adc_ports.p32_data[1] :> inp_val; // Get new input
        tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
      // printhexln(tmp_val);
        if(config_ == 0) tmp_val = tmp_val >> SHIFTING_BITS;
        else tmp_val = tmp_val >> (SHIFTING_BITS+1);
        tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
        out_b = (int)tmp_val;

        t :> tout;

#pragma ordered
         select {
             case iADC.get_adc_measurements(unsigned char port_id, unsigned char config) -> {int phaseB, int phaseC, int sampling_time}:
                 config_ = config;
                 if (port_id > 6 || port_id < 1) port_id = 1;
                 mux_config = (config << 3) | (port_id - 1);
                 adc_ports.p4_mux <: mux_config;
                 phaseB = out_a;
                 phaseC = out_b;
                 sampling_time = tout-tin;
             break;

             default:  break;
         }// end of select
    } // while (do_loop)

    printstrln("Terminated loop");
    printstrln("\n                                             ADC Server Ends");
} // foc_adc_7265_triggered

void run_adc_AD7256(server interface ADC iADC, server interface ADC_trigger i_ADC_trigger, adc_ports_t &adc_ports)
{
    printstrln("                                           ADC Server Starts");
    unsigned time_stamp; // Time stamp
    //        A/B | AI
    int adc_data[2][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0}};
    unsigned inp_val = 0, tmp_val = 0;
    int out_a = 0, out_b= 0;
    timer t;
    unsigned tin = 0, tout = 0;
    int sampling_time_ = 0;

    configure_adc_ports_7265( adc_ports.p32_data, adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    while(1){

    #pragma ordered
        select {
            case i_ADC_trigger.trigger_measurement(unsigned char port_id, unsigned char config):
                    t :> tin;
                    if (port_id > 6 || port_id < 1) port_id = 1;
                    unsigned char mux_config = (config << 3) | (port_id - 1);
                    adc_ports.p4_mux <: mux_config;
                    clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
                    clearbuf( adc_ports.p32_data[1] );

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
                    out_a = (int)tmp_val;

                    // Get data from port b
                    endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
                    adc_ports.p32_data[1] :> inp_val; // Get new input
                    tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
                    if(config == 0) tmp_val = tmp_val >> SHIFTING_BITS;
                    else tmp_val = tmp_val >> (SHIFTING_BITS+1);
                    tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                    out_b = (int)tmp_val;

                    t :> tout;

                    // Store data
                    adc_data[0][port_id] = out_a;
                    adc_data[1][port_id] = out_b;
                    sampling_time_ = (int)(tout-tin);
                break;

            case iADC.get_adc_measurements(unsigned char port_id, unsigned char config) -> {int adc_A, int adc_B, int sampling_time}:
                        adc_A = adc_data[0][port_id];
                        adc_B = adc_data[1][port_id];
                        sampling_time = sampling_time_;
                 break;

            default:
                break;
        }//eof select
    }//eof while
}
/*****************************************************************************/
// adc_7265.xc

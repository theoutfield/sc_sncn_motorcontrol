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
#include <protection.h>
//#include <xscope.h>
#include <motor_control_interfaces.h>


#define SHIFTING_BITS   1

////Takes 8ms for the filter to react to a current spike
//#define OVERCURRENT_SAMPLES 20

//#ifdef DISPLAY_MAX_CURRENTS
//#include <xscope.h>
//#endif





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



void adc_ad7256(
        interface ADCInterface server iADC[2],
        AD7265Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog)
{
    timer t;
    unsigned time=0, time_stamp=0;

    unsigned inp_val = 0, tmp_val = 0;

    configure_adc_ports_7265( adc_ports.p32_data[0], adc_ports.p32_data[1], adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    while(1)
    {
        #pragma ordered
        select
        {
        case iADC[int i].status() -> {int status}:
                status = ACTIVE;
                break;

        case iADC[int i].set_channel(unsigned short channel_config):
                adc_ports.p4_mux <: channel_config;
                clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
                clearbuf( adc_ports.p32_data[1] );
                t :> time;
                t when timerafter (time + 1000) :> void;//10 us of wait
                break;

        case iADC[int i].sample_and_send()-> {int out_a, int out_b}:

                clearbuf( adc_ports.p32_data[0] );      // Clear the buffers used by the input ports.
                clearbuf( adc_ports.p32_data[1] );

                adc_ports.p1_ready <: 1 @ time_stamp;   // Switch ON input reads (and ADC conversion)
                time_stamp += (ADC_TOTAL_BITS+2);       // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
                adc_ports.p1_ready @ time_stamp <: 0;   // Switch OFF input reads, (and ADC conversion)

                sync( adc_ports.p1_ready );             // Wait until port has completed any pending outputs

                // Get data from port a
                endin( adc_ports.p32_data[0] );   // End the previous input on this buffered port
                adc_ports.p32_data[0] :> inp_val; // Get new input
                tmp_val = bitrev( inp_val );      // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_a = (int)tmp_val;

                // Get data from port b
                endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
                adc_ports.p32_data[1] :> inp_val; // Get new input
                tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_b = (int)tmp_val;
                break;

        case iADC[int i].set_protection_limits(int i_max, int i_ratio, int v_dc_max, int v_dc_min):
                break;

        case iADC[int i].get_all_measurements() -> {int phaseB_out, int phaseC_out, int V_dc_out, int torque_out, int fault_code_out}:
                break;

        case iADC[int i].reset_faults():
                break;

        }//eof select
    }//eof while
}

void adc_ad7256_fixed_channel(
        interface ADCInterface server iADC[2],
        AD7265Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog)
{

    timer t;
    unsigned int time;

    unsigned time_stamp; // Time stamp

    int i_max   =100 ;
    int v_dc_max=100 ;
    int v_dc_min=0   ;

    int fault_code=NO_FAULT;

    unsigned inp_val = 0, tmp_val = 0;
    int i=0;

    int out_a=0, out_b=0;
    int V_dc=0;

    int flag=0;

    int I_a=0;
    int I_b=0;
    int I_c=0;
    int current_limit = i_max * 20;

    int torque=0;

    int j=0;
    int selected_channel=0;
    unsigned short channel_config[10] = {SGL_A1_B1, SGL_A2_B2, SGL_A3_B3, SGL_A4_B4, SGL_A5_B5, SGL_A6_B6, DIFF_A1A2_B1B2, DIFF_A3A4_B3B4, DIFF_A5A6_B5B6};
    int OUT_A[10], OUT_B[10];

    //proper task startup
    t :> time;
    t when timerafter (time + (3000*20*250)) :> void;


    configure_adc_ports_7265(adc_ports.p32_data[0], adc_ports.p32_data[1], adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    /* Read the ADC several times to assure the read values are correct.
     * Overwise it could happen that we run into overprotection.
     */
    for (i=0;i<150;i++)
    {
        adc_ports.p4_mux <: SGL_A1_B1;      //mux_config;
        clearbuf( adc_ports.p32_data[0] );  // Clear the buffers used by the input ports.
        clearbuf( adc_ports.p32_data[1] );
        adc_ports.p1_ready <: 1 @ time_stamp;   // Switch ON input reads (and ADC conversion)
        time_stamp += (ADC_TOTAL_BITS+2);       // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
        adc_ports.p1_ready @ time_stamp <: 0;   // Switch OFF input reads, (and ADC conversion)

        sync( adc_ports.p1_ready );             // Wait until port has completed any pending outputs

        // Get data from port a
        endin( adc_ports.p32_data[0] );         // End the previous input on this buffered port
        adc_ports.p32_data[0] :> inp_val;       // Get new input

        // Get data from port b
        endin( adc_ports.p32_data[1] );         // End the previous input on this buffered port
        adc_ports.p32_data[1] :> inp_val;   // Get new input
    }

    while(1)
    {
#pragma ordered
        select
        {

        case iADC[int i].status() -> {int status}:
                status = ACTIVE;
                break;

        case iADC[int i].set_protection_limits(int i_max_in, int i_ratio, int v_dc_max_in, int v_dc_min_in):
                i_max=i_max_in;
                v_dc_max=v_dc_max_in;
                v_dc_min=v_dc_min_in;
                current_limit = i_max * i_ratio;
                break;

        case iADC[int i].get_all_measurements() -> {int phaseB_out, int phaseC_out, int V_dc_out, int torque_out, int fault_code_out}:

                adc_ports.p4_mux <: SGL_A1_B1;      //mux_config;
                clearbuf( adc_ports.p32_data[0] );  //Clear the buffers used by the input ports.
                clearbuf( adc_ports.p32_data[1] );
                adc_ports.p1_ready <: 1 @ time_stamp;   // Switch ON input reads (and ADC conversion)
                time_stamp += (ADC_TOTAL_BITS+2);       // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
                adc_ports.p1_ready @ time_stamp <: 0;   // Switch OFF input reads, (and ADC conversion)

                sync( adc_ports.p1_ready );             // Wait until port has completed any pending outputs

                // Get data from port a
                endin( adc_ports.p32_data[0] );         // End the previous input on this buffered port
                adc_ports.p32_data[0] :> inp_val;       // Get new input
                tmp_val = bitrev( inp_val );            // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_a = (int)tmp_val;

                // Get data from port b
                endin( adc_ports.p32_data[1] );         // End the previous input on this buffered port
                adc_ports.p32_data[1] :> inp_val;       // Get new input
                tmp_val = bitrev( inp_val );            // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_b = (int)tmp_val;

                phaseB_out = current_sensor_config.sign_phase_b * (out_a - 2048);
                phaseC_out = current_sensor_config.sign_phase_c * (out_b - 2048);

                I_b = phaseB_out;
                I_c = phaseC_out;
                I_a = -I_b-I_c;

                if( I_a<(-current_limit) || current_limit<I_a)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_A);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_A;
                }

                if( I_b<(-current_limit) || current_limit<I_b)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_B);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_B;
                }

                if( I_c<(-current_limit) || current_limit<I_c)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_C);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_C;
                }

                V_dc_out = V_dc;
                torque_out = torque;
                fault_code_out=fault_code;

                flag=1;
                break;

        case iADC[int i].set_channel(unsigned short channel_in):
                selected_channel = channel_in;
                break;

        case iADC[int i].sample_and_send()-> {int out_a, int out_b}:
                for(int k=0;k<10;k++)
                {
                    if(selected_channel == channel_config[k])
                    {
                        out_a = OUT_A[k];
                        out_b = OUT_B[k];
                    }
                }
                break;

        case iADC[int i].reset_faults():
                I_a=0;
                I_b=0;
                I_c=0;
                torque=0;
                V_dc=(v_dc_min+v_dc_max)/2;

                fault_code=NO_FAULT;
                flag=0;

                i_watchdog.reset_faults();
                break;

        default:
            break;
        }//eof select

        if(flag==1)
        {
            j++;
            if(j==9) j=0;

            adc_ports.p4_mux <: channel_config[j];
            t :> time;
            t when timerafter (time + 500) :> void;//5 us of wait

            clearbuf( adc_ports.p32_data[0] );  //Clear the buffers used by the input ports.
            clearbuf( adc_ports.p32_data[1] );
            adc_ports.p1_ready <: 1 @ time_stamp;   // Switch ON input reads (and ADC conversion)
            time_stamp += (ADC_TOTAL_BITS+2);       // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
            adc_ports.p1_ready @ time_stamp <: 0;   // Switch OFF input reads, (and ADC conversion)

            sync( adc_ports.p1_ready );             // Wait until port has completed any pending outputs

            // Get data from port a
            endin( adc_ports.p32_data[0] );         // End the previous input on this buffered port
            adc_ports.p32_data[0] :> inp_val;       // Get new input
            tmp_val = bitrev( inp_val );            // Reverse bit order. WARNING. Machine dependent
            tmp_val = tmp_val >> (SHIFTING_BITS+1);
            tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
            OUT_A[j] = (int)tmp_val;

            // Get data from port b
            endin( adc_ports.p32_data[1] );         // End the previous input on this buffered port
            adc_ports.p32_data[1] :> inp_val;       // Get new input
            tmp_val = bitrev( inp_val );            // Reverse bit order. WARNING. Machine dependent
            tmp_val = tmp_val >> (SHIFTING_BITS+1);
            tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
            OUT_B[j] = (int)tmp_val;


            V_dc = OUT_A[1]/56;

            if (V_dc<v_dc_min)
            {
                i_watchdog.protect(UNDER_VOLTAGE);
                if(fault_code==0) fault_code=UNDER_VOLTAGE;
            }

            if (v_dc_max<V_dc)
            {
                i_watchdog.protect(OVER_VOLTAGE);
                if(fault_code==0) fault_code=OVER_VOLTAGE;
            }

            torque = OUT_A[2]-OUT_B[2];


            adc_ports.p4_mux <: SGL_A1_B1;
            t :> time;
            t when timerafter (time + 500) :> void;//5 us of wait

            for (i=0;i<=5;i++)
            {
                clearbuf( adc_ports.p32_data[0] ); // Clear the buffers used by the input ports.
                clearbuf( adc_ports.p32_data[1] );
                adc_ports.p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
                time_stamp += (ADC_TOTAL_BITS+2); // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
                adc_ports.p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)

                sync( adc_ports.p1_ready ); // Wait until port has completed any pending outputs

                // Get data from port a
                endin( adc_ports.p32_data[0] );   // End the previous input on this buffered port
                adc_ports.p32_data[0] :> inp_val; // Get new input
                tmp_val = bitrev( inp_val );      // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_a = (int)tmp_val;

                // Get data from port b
                endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
                adc_ports.p32_data[1] :> inp_val; // Get new input
                tmp_val = bitrev( inp_val );    // Reverse bit order. WARNING. Machine dependent
                tmp_val = tmp_val >> (SHIFTING_BITS+1);
                tmp_val = (short)(tmp_val & ADC_MASK);  // Mask out active bits and convert to signed word
                out_b = (int)tmp_val;

                I_b = out_a - 2048;
                I_c = out_b - 2048;
                I_a = -I_b-I_c;

                if( I_a<(-current_limit) || current_limit<I_a)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_A);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_A;
                }

                if( I_b<(-current_limit) || current_limit<I_b)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_B);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_B;
                }

                if( I_c<(-current_limit) || current_limit<I_c)
                {
                    i_watchdog.protect(OVER_CURRENT_PHASE_C);
                    if(fault_code==0) fault_code=OVER_CURRENT_PHASE_C;
                }
            }

            flag=0;

        }
    }//eof while(1)


}


/*****************************************************************************/
// adc_7265.xc

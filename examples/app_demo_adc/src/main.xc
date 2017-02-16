/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

#include <adc_service.h>
#include <motor_control_interfaces.h>
#include <demo_adc.h>

ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

/**
 * @brief Service to sample analogue inputs of ADC module
 *
 * @param iADC[2]               Interface to communicate with clients and send the measured values
 * @param adc_ports             Structure type to manage the AD7265 ADC chip.
 * @param current_sensor_config Structure type to calculate the proper sign (positive/negative) of sampled phase currents
 * @param i_watchdog            Interface to communicate with watchdog service
 * @param operational_mode      Integer type to select between SINGLE_ENDED/FULLY_DIFFERENTIAL modes
 *
 * @return void
 */
void adc_ad7265_single_shot(ADCPorts &adc_ports, interface ADCInterface server iADC[2], interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec, int operational_mode)
{
    timer t;
    unsigned int time;
    unsigned time_stamp; // Time stamp

    unsigned inp_val = 0, tmp_val = 0;

    int out_a=0, out_b=0;
    //proper task startup
    t :> time;
    t when timerafter (time + (3000*20*250)) :> void;

//    configure_adc_ports_7265(adc_ports.p32_data[0], adc_ports.p32_data[1], adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready, adc_ports.p4_mux ); // Configure all ADC data ports

    while(1)
    {
#pragma ordered
        select
        {

        case iADC[int i].get_channel(unsigned short channel_in)-> {int output_a, int output_b}:
//                adc_ports.p4_mux <: channel_in;
//                t :> time;
//                t when timerafter (time + 500) :> void;//5 us of wait
//
//                clearbuf( adc_ports.p32_data[0] ); //Clear the buffers used by the input ports.
//                clearbuf( adc_ports.p32_data[1] );
//                adc_ports.p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
//                time_stamp += (ADC_TOTAL_BITS+2); // Allows sample-bits to be read on buffered input ports TODO: Check if +2 is cool enough and why
//                adc_ports.p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)
//
//                sync( adc_ports.p1_ready ); // Wait until port has completed any pending outputs
//
//                // Get data from port a
//                endin( adc_ports.p32_data[0] ); // End the previous input on this buffered port
//                adc_ports.p32_data[0] :> inp_val; // Get new input
//                tmp_val = bitrev( inp_val ); // Reverse bit order. WARNING. Machine dependent
//                tmp_val = tmp_val >> (SHIFTING_BITS+1);
//                tmp_val = (short)(tmp_val & ADC_MASK); // Mask out active bits and convert to signed word
//                output_a = (int)tmp_val;
//
//                // Get data from port b
//                endin( adc_ports.p32_data[1] ); // End the previous input on this buffered port
//                adc_ports.p32_data[1] :> inp_val; // Get new input
//                tmp_val = bitrev( inp_val ); // Reverse bit order. WARNING. Machine dependent
//                tmp_val = tmp_val >> (SHIFTING_BITS+1);
//                tmp_val = (short)(tmp_val & ADC_MASK); // Mask out active bits and convert to signed word
//                output_b = (int)tmp_val;
                break;

        case iADC[int i].status() -> {int status}:
                break;

        case iADC[int i].set_protection_limits_and_analogue_input_configs(
                int i_max_in, int i_ratio_in, int v_dc_max_in, int v_dc_min_in):
                break;

        case iADC[int i].get_all_measurements() -> {
            int phaseB_out, int phaseC_out,
            int V_dc_out, int I_dc_out, int Temperature_out,
            int analogue_input_a_1, int analogue_input_a_2,
            int analogue_input_b_1, int analogue_input_b_2,
            int fault_code_out}:
            break;

        case iADC[int i].reset_faults():
                break;

        default:
            break;
        }//eof select
    }//eof while(1)
}

int main(void)
{
    // ADC interface
    interface ADCInterface i_adc[2];
    interface WatchdogInterface i_watchdog[2];


    par
    {
        on tile[APP_TILE]:
        {
            demo_adc(i_adc[1]);
        }

        on tile[IFM_TILE]:
        {
            adc_ad7265_single_shot(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, SINGLE_ENDED);
        }
    }

    return 0;
}

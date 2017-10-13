/**
 * @file adc_server_ad7949.xc
 * @brief ADC Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>
#include <stdint.h>
#include <xclib.h>
#include <refclk.h>
#include <adc_ad7949.h>

/**
 * @brief Define bit masks to distinguish if a single bit is 0/1 in a 14 bit binary variable.
 */
#define BIT13 0x00002000
#define BIT12 0x00001000
#define BIT11 0x00000800
#define BIT10 0x00000400
#define BIT09 0x00000200
#define BIT08 0x00000100
#define BIT07 0x00000080
#define BIT06 0x00000040
#define BIT05 0x00000020
#define BIT04 0x00000010
#define BIT03 0x00000008
#define BIT02 0x00000004
#define BIT01 0x00000002
#define BIT0  0x00000001

/**
 * @brief Configure all ADC data ports
 *
 * @param clk                       XMOS internal clock
 * @param p_sclk_conv_mosib_mosia   32-bit buffered port to commiunicate with AD7949 through SPI
 * @param p_data_a                  32-bit buffered port to recieve the data from AD7949
 * @param p_data_b                  32-bit buffered port to recieve the data from AD7949
 *
 * @return void
 */
static void configure_adc_ports(
        clock clk,
        buffered out port:32 p_sclk_conv_mosib_mosia,
        in buffered port:32 p_data_a,
        in buffered port:32 p_data_b)
{
    /* SCLK period >= 22ns (45.45 MHz)
       clk needs to be configured twice as fast as the required SCLK frequency */
    configure_clock_rate_at_most(clk, 250, 7); // 83.3  --  < (2*45.45)

    /* when idle, keep clk and mosi low, conv high */
    configure_out_port(p_sclk_conv_mosib_mosia, clk, 0b0100);
    configure_in_port(p_data_a, clk);
    configure_in_port(p_data_b, clk);
    start_clock(clk);
}// configure_adc_ports

/**
 * @brief Convert the output (serial) data of the adc in to unsigned value
 *
 * @param raw       Type unsigned value which is sent from AD7949 through SPI communication
 *
 * @return unsigned Converted Digital value of AD7949
 */
static inline unsigned convert(unsigned raw)
{
    unsigned int data;

    /* raw == 0b xxxx aabb ccdd eeff ...
       we read every data bit twice because of port clock setting */

    raw = bitrev(raw);
    data  = raw & 0x06000000;
    data >>= 2;
    data |= raw & 0x00600000;
    data >>= 2;
    data |= raw & 0x00060000;
    data >>= 2;
    data |= raw & 0x00006000;
    data >>= 2;
    data |= raw & 0x00000600;
    data >>= 2;
    data |= raw & 0x00000060;
    data >>= 2;
    data |= raw & 0x00000006;
    data >>= 1;
    return data;
}// convert

/**
 * @brief Demo service to show how AD7949 can be used.
 *
 * @param adc_ports             Structure type to manage the AD7265 ADC chip.
 * @param iADC[2]               Interface to communicate with clients and send the measured values
 *
 * @return void
 */
void adc_ad7949_service_demo(
        AD7949Ports &adc_ports,
        interface ADCInterface server iADC[2])
{
    /*
     * Configuration Register Description
     *
     * bit(s)   name    Description
     *
     *  13      CFG     Configuration udpate
     *  12      INCC    Input channel configuration
     *  11      INCC    Input channel configuration
     *  10      INCC    Input channel configuration
     *  09      INx     Input channel selection bit 2 0..7
     *  08      INx     Input channel selection bit 1
     *  07      INx     Input channel selection bit 0
     *  06      BW      Select bandwidth for low-pass filter
     *  05      REF     Reference/buffer selection
     *  04      REF     Reference/buffer selection
     *  03      REF     Reference/buffer selection
     *  02      SEQ     Channel sequencer. Allows for scanning channels in an IN0 to IN[7:0] fashion.
     *  01      SEQ     Channel sequencer
     *  00      RB      Read back the CFG register.
     *
     * Initialize the "Configuration Register":
     *
     * Overwrite configuration update | unipolar, referenced to GND | Motor current (ADC Channel IN0)| full Bandwidth | Internal reference, REF = 4,096V, temp enabled;
     * bit[13] = 1                      bits[12:10]: 111              bits[9:7] 000                    bit[6] 1         bits[5:3] 001
     *  Disable Sequencer | Do not read back contents of configuration
     *  bits[2:1] 00        bit[0] 1
     *
     */
    unsigned int ad7949_config       =   0b11110001001001;

    unsigned int adc_data_a=0;
    unsigned int adc_data_b=0;

    unsigned int data_raw_a;
    unsigned int data_raw_b;

    configure_adc_ports(adc_ports.clk, adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b);

    while (1)
    {
#pragma ordered
        select
        {
        case iADC[int i].get_channel(unsigned short channel_config_in)-> {int output_a, int output_b}:

                ad7949_config = channel_config_in;

                configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);

#pragma unsafe arrays
                int bits[4];

                bits[0]=0x80808000;
                if(ad7949_config & BIT13)
                    bits[0] |= 0x0000B300;
                if(ad7949_config & BIT12)
                    bits[0] |= 0x00B30000;
                if(ad7949_config & BIT11)
                    bits[0] |= 0xB3000000;

                bits[1]=0x80808080;
                if(ad7949_config & BIT10)
                    bits[1] |= 0x000000B3;
                if(ad7949_config & BIT09)
                    bits[1] |= 0x0000B300;
                if(ad7949_config & BIT08)
                    bits[1] |= 0x00B30000;
                if(ad7949_config & BIT07)
                    bits[1] |= 0xB3000000;

                bits[2]=0x80808080;
                if(ad7949_config & BIT06)
                    bits[2] |= 0x000000B3;
                if(ad7949_config & BIT05)
                    bits[2] |= 0x0000B300;
                if(ad7949_config & BIT04)
                    bits[2] |= 0x00B30000;
                if(ad7949_config & BIT03)
                    bits[2] |= 0xB3000000;

                bits[3]=0x00808080;
                if(ad7949_config & BIT02)
                    bits[3] |= 0x000000B3;
                if(ad7949_config & BIT01)
                    bits[3] |= 0x0000B300;
                if(ad7949_config & BIT0)
                    bits[3] |= 0x00B30000;

                for(int i=0;i<=3;i++)
                {
                    stop_clock(adc_ports.clk);
                    clearbuf(adc_ports.data_a);
                    clearbuf(adc_ports.data_b);
                    clearbuf(adc_ports.sclk_conv_mosib_mosia);
                    adc_ports.sclk_conv_mosib_mosia <: bits[0];
                    start_clock(adc_ports.clk);

                    adc_ports.sclk_conv_mosib_mosia <: bits[1];
                    adc_ports.sclk_conv_mosib_mosia <: bits[2];
                    adc_ports.sclk_conv_mosib_mosia <: bits[3];

                    sync(adc_ports.sclk_conv_mosib_mosia);
                    stop_clock(adc_ports.clk);

                    configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);

                    adc_ports.data_a :> data_raw_a;
                    adc_data_a = convert(data_raw_a);
                    adc_ports.data_b :> data_raw_b;
                    adc_data_b = convert(data_raw_b);

                    configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);

                    output_a = ((int) adc_data_a);
                    output_b = ((int) adc_data_b);
                }
                break;

        case iADC[int i].status() -> {int status}:
                break;

        case iADC[int i].set_protection_limits(
                int limit_oc, int limit_ov, int limit_uv, int limit_ot):
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
        }
    }
}// adc_ad7949_service_demo

/**
 * @brief Service to sample analogue inputs of ADC module
 *
 * @param iADC[2]               Interface to communicate with clients and send the measured values
 * @param adc_ports             Structure type to manage the AD7949 ADC chip.
 * @param current_sensor_config Structure type to calculate the proper sign (positive/negative) of sampled phase currents
 * @param i_watchdog            Interface to communicate with watchdog service
 * @param operational_mode      Reserved
 *
 * @return void
 */
void adc_ad7949(
        interface ADCInterface server iADC[2],
        AD7949Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog, int operational_mode, int ifm_tile_usec)
{
    timer t;
    unsigned int time;
    unsigned int t_start=0;

    /*
     * Configuration Register Description
     *
     * bit(s)   name    Description
     *
     *  13      CFG     Configuration udpate
     *  12      INCC    Input channel configuration
     *  11      INCC    Input channel configuration
     *  10      INCC    Input channel configuration
     *  09      INx     Input channel selection bit 2 0..7
     *  08      INx     Input channel selection bit 1
     *  07      INx     Input channel selection bit 0
     *  06      BW      Select bandwidth for low-pass filter
     *  05      REF     Reference/buffer selection
     *  04      REF     Reference/buffer selection
     *  03      REF     Reference/buffer selection
     *  02      SEQ     Channel sequencer. Allows for scanning channels in an IN0 to IN[7:0] fashion.
     *  01      SEQ     Channel sequencer
     *  00      RB      Read back the CFG register.
     *
     * Initialize the "Configuration Register":
     *
     * Overwrite configuration update | unipolar, referenced to GND | Motor current (ADC Channel IN0)| full Bandwidth | Internal reference, REF = 4,096V, temp enabled;
     * bit[13] = 1                      bits[12:10]: 111              bits[9:7] 000                    bit[6] 1         bits[5:3] 001
     *  Disable Sequencer | Do not read back contents of configuration
     *  bits[2:1] 00        bit[0] 1
     *
     */
    const unsigned int adc_config_mot=   0b11110001001001;
    unsigned int ad7949_config       =   0b11110001001001;

    unsigned int adc_data_a=0;
    unsigned int adc_data_b=0;
    unsigned int hdw_delay = 1400;

    unsigned int data_raw_a;
    unsigned int data_raw_b;

    int OUT_A[4], OUT_B[4];
    int j=0;

    const unsigned int channel_config[4] = {
            AD7949_CHANNEL_0,   // ADC Channel 2, unipolar, referenced to GND voltage and current
            AD7949_CHANNEL_2,   // ADC Channel 2, unipolar, referenced to GND voltage and current
            AD7949_CHANNEL_4,   // ADC Channel 4, unipolar, referenced to GND
            AD7949_CHANNEL_5};  // ADC Channel 5, unipolar, referenced to GND

    int i_calib_a = 5000, i_calib_b = 5000;

    int data_updated=0;

    int dc_value=2617;

    int v_dc_max=100;
    int v_dc_min=0;
    int current_limit = 100;
    int protection_counter=0;

    int fault_code=NO_FAULT;

    //proper task startup
    t :> time;
    t when timerafter (time + (3000*20*250)) :> void;

    configure_adc_ports(adc_ports.clk, adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b);

    while (1)
    {
#pragma ordered
        select
        {

        case iADC[int i].get_channel(unsigned short channel_in)-> {int output_a, int output_b}:
                break;

        case iADC[int i].status() -> {int status}:
                status = ACTIVE;
                break;

        case iADC[int i].set_protection_limits(
                int limit_oc, int limit_ov, int limit_uv, int limit_ot):
                current_limit = limit_oc;
                v_dc_max      = limit_ov;
                v_dc_min      = limit_uv;
                break;

        case iADC[int i].get_all_measurements() -> {
            int phaseB_out, int phaseC_out,
            int V_dc_out, int I_dc_out, int Temperature_out,
            int analogue_input_a_1, int analogue_input_a_2,
            int analogue_input_b_1, int analogue_input_b_2,
            int fault_code_out}:

            t :> t_start;
            phaseB_out = (current_sensor_config.sign_phase_b * (OUT_A[0] - i_calib_a))/20;
            phaseC_out = (current_sensor_config.sign_phase_c * (OUT_B[0] - i_calib_b))/20;

            V_dc_out=OUT_A[AD_7949_VMOT_DIV_I_MOT]-dc_value;

            if((5000<protection_counter) && (fault_code==NO_FAULT))
            {
                if(( phaseB_out<(-current_limit) || current_limit<phaseB_out))
                {
                    i_watchdog.protect(WD_OVER_CURRENT_PHASE_B);
                    fault_code=DEVICE_INTERNAL_CONTINOUS_OVER_CURRENT_NO_1;
                }

                if(( phaseC_out<(-current_limit) || current_limit<phaseC_out))
                {
                    i_watchdog.protect(WD_OVER_CURRENT_PHASE_C);
                    fault_code=DEVICE_INTERNAL_CONTINOUS_OVER_CURRENT_NO_1;
                }

                if (V_dc_out<v_dc_min)
                {
                    i_watchdog.protect(WD_UNDER_VOLTAGE);
                    fault_code=UNDER_VOLTAGE_NO_1;
                }

                if (v_dc_max<V_dc_out)
                {
                    i_watchdog.protect(WD_OVER_VOLTAGE);
                    fault_code=OVER_VOLTAGE_NO_1;
                }
            }
            I_dc_out=OUT_B[AD_7949_VMOT_DIV_I_MOT];
            analogue_input_a_1 = OUT_A[AD_7949_EXT_A0_N_EXT_A1_N];
            analogue_input_b_1 = OUT_B[AD_7949_EXT_A0_N_EXT_A1_N];
            analogue_input_a_2 = OUT_A[AD_7949_EXT_A0_P_EXT_A1_P];
            analogue_input_b_2 = OUT_B[AD_7949_EXT_A0_P_EXT_A1_P];
            Temperature_out=0;
            fault_code_out=fault_code;
            data_updated=1;
            break;

        case iADC[int i].reset_faults():
                fault_code=NO_FAULT;
                data_updated=0;
                protection_counter=0;
                i_watchdog.reset_faults();
                break;
        default:
            break;
        }

        if(data_updated==1)
        {
            if(protection_counter<10000) protection_counter++;

            t when timerafter (t_start + hdw_delay) :> void;
            for(j=AD_7949_EXT_A0_P_EXT_A1_P;AD_7949_IB_IC<=j;j--)
            {
                ad7949_config = channel_config[j];

                configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);

#pragma unsafe arrays
                int bits[4];

                bits[0]=0x80808000;
                if(ad7949_config & BIT13)
                    bits[0] |= 0x0000B300;
                if(ad7949_config & BIT12)
                    bits[0] |= 0x00B30000;
                if(ad7949_config & BIT11)
                    bits[0] |= 0xB3000000;

                bits[1]=0x80808080;
                if(ad7949_config & BIT10)
                    bits[1] |= 0x000000B3;
                if(ad7949_config & BIT09)
                    bits[1] |= 0x0000B300;
                if(ad7949_config & BIT08)
                    bits[1] |= 0x00B30000;
                if(ad7949_config & BIT07)
                    bits[1] |= 0xB3000000;

                bits[2]=0x80808080;
                if(ad7949_config & BIT06)
                    bits[2] |= 0x000000B3;
                if(ad7949_config & BIT05)
                    bits[2] |= 0x0000B300;
                if(ad7949_config & BIT04)
                    bits[2] |= 0x00B30000;
                if(ad7949_config & BIT03)
                    bits[2] |= 0xB3000000;

                bits[3]=0x00808080;
                if(ad7949_config & BIT02)
                    bits[3] |= 0x000000B3;
                if(ad7949_config & BIT01)
                    bits[3] |= 0x0000B300;
                if(ad7949_config & BIT0)
                    bits[3] |= 0x00B30000;

                for(int i=0;i<=4;i++)
                {
                    stop_clock(adc_ports.clk);
                    clearbuf(adc_ports.data_a);
                    clearbuf(adc_ports.data_b);
                    clearbuf(adc_ports.sclk_conv_mosib_mosia);
                    adc_ports.sclk_conv_mosib_mosia <: bits[0];
                    start_clock(adc_ports.clk);

                    adc_ports.sclk_conv_mosib_mosia <: bits[1];
                    adc_ports.sclk_conv_mosib_mosia <: bits[2];
                    adc_ports.sclk_conv_mosib_mosia <: bits[3];

                    sync(adc_ports.sclk_conv_mosib_mosia);
                    stop_clock(adc_ports.clk);

                    configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);

                    delay_ticks(ADC7949_DATA_VALID_DELAY);
                    adc_ports.data_a :> data_raw_a;
                    adc_data_a = convert(data_raw_a);
                    adc_ports.data_b :> data_raw_b;
                    adc_data_b = convert(data_raw_b);

                    configure_out_port(adc_ports.sclk_conv_mosib_mosia, adc_ports.clk, 0b0100);
                }

                OUT_A[j] = ((int) adc_data_a);
                OUT_B[j] = ((int) adc_data_b);
            }
            data_updated=0;
        }
    }
}// adc_ad7949


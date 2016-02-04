/**
 * @file adc_server_ad7949.xc
 * @brief ADC Server
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <xs1.h>
#include <stdint.h>
#include <xclib.h>
#include <refclk.h>
#include <print.h>
#include <adc_ad7949.h>

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

#define ADC_CALIB_POINTS 64
#define Factor 6

void output_adc_config_data(clock clk, in buffered port:32 p_data_a, in buffered port:32 p_data_b,
                            buffered out port:32 p_adc, int adc_cfg_data)
{
#pragma unsafe arrays
    int bits[4];

    bits[0]=0x80808000;
    if(adc_cfg_data & BIT13)
        bits[0] |= 0x0000B300;
    if(adc_cfg_data & BIT12)
        bits[0] |= 0x00B30000;
    if(adc_cfg_data & BIT11)
        bits[0] |= 0xB3000000;

    bits[1]=0x80808080;
    if(adc_cfg_data & BIT10)
        bits[1] |= 0x000000B3;
    if(adc_cfg_data & BIT09)
        bits[1] |= 0x0000B300;
    if(adc_cfg_data & BIT08)
        bits[1] |= 0x00B30000;
    if(adc_cfg_data & BIT07)
        bits[1] |= 0xB3000000;

    bits[2]=0x80808080;
    if(adc_cfg_data & BIT06)
        bits[2] |= 0x000000B3;
    if(adc_cfg_data & BIT05)
        bits[2] |= 0x0000B300;
    if(adc_cfg_data & BIT04)
        bits[2] |= 0x00B30000;
    if(adc_cfg_data & BIT03)
        bits[2] |= 0xB3000000;

    bits[3]=0x00808080;
    if(adc_cfg_data & BIT02)
        bits[3] |= 0x000000B3;
    if(adc_cfg_data & BIT01)
        bits[3] |= 0x0000B300;
    if(adc_cfg_data & BIT0)
        bits[3] |= 0x00B30000;

    stop_clock(clk);
    clearbuf(p_data_a);
    clearbuf(p_data_b);
    clearbuf(p_adc);
    p_adc <: bits[0];
    start_clock(clk);

    p_adc <: bits[1];
    p_adc <: bits[2];
    p_adc <: bits[3];

    sync(p_adc);
    stop_clock(clk);
}

static void configure_adc_ports(clock clk,
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
}

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
}

/*
 * ADC CONFIGURAION HOWTO
 *
 * Serialization: LSB first
 *
 * adc_config: !! reverse bit-order compared to AD7949 datasheet !!
 * Bit    Name  Description
 * 32:14  0     unused
 * 13     RB    Read back CFG reg       (  1 => no)
 * 12:11  SEQ   Sequencer               ( 00 => disable)
 * 10:8   REF   Reference selection     (100 => internal)
 * 7      BW    LPF bandwidth           (  1 => full)
 * 6:4    INx   Input channel selection (000 => CH0)
 * 3:1    INCC  Input channel config    (111 => unipolar, referenced to GND)
 * 0      CFG   Config update           (  1 => overwrite cfg reg)
 * -1     --PADDING-- (delay 1 clk cycle)
 * => adc_config = 0b100100100011110
 */

#pragma unsafe arrays
static void adc_ad7949_singleshot( buffered out port:32 p_sclk_conv_mosib_mosia,
                                   in buffered port:32 p_data_a,
                                   in buffered port:32 p_data_b,
                                   clock clk,
                                   const unsigned int adc_config_mot,
                                   const unsigned int adc_config_other[],
                                   const unsigned int delay,
                                   timer t,
                                   unsigned int adc_data_a[],
                                   unsigned int adc_data_b[],
                                   unsigned short &adc_index )
{
    unsigned int ts;
    unsigned int data_raw_a;
    unsigned int data_raw_b;

    /* Reading/Writing after conversion (RAC)
       Read previous conversion result
       Write CFG for next conversion */
#define SPI_IDLE   configure_out_port(p_sclk_conv_mosib_mosia, clk, 0b0100)

    // CONGIG__other_n1     CFG_Imotx_x1       |CONGIG__other_n2     CFG_Imotx_x2     |CONGIG__other_n3     CFG_Imotx_x3       |
    // CONVERT_null         CONVERT_other_n1   |CONVERT_Imot_x1      CONVERT_other_n2 |CONVERT_Imot_x2      CONVERT_other_n3   |
    // READOUT_null         READOUT_other_null |READOUT_other_n1     READOUT_Imot_x1  |READOUT_other_n2     READOUT_Imot_x2    |
    // -----------
    // iIndexADC        0  			1			       2			      3
    // readout       extern   	temperature		current-voltage			extern

    SPI_IDLE;

    output_adc_config_data(clk, p_data_a, p_data_b, p_sclk_conv_mosib_mosia, adc_config_other[adc_index]);
    SPI_IDLE;

    t :> ts;
    p_data_a :> data_raw_a;
    adc_data_a[adc_index] = convert(data_raw_a);
    p_data_b :> data_raw_b;
    adc_data_b[adc_index] = convert(data_raw_b);
    adc_index++;
    adc_index &= 0x3;
    t when timerafter(ts + delay) :> ts;

    output_adc_config_data(clk, p_data_a, p_data_b, p_sclk_conv_mosib_mosia, adc_config_mot);
    SPI_IDLE;


    p_data_a :> data_raw_a;
    adc_data_a[4] = convert(data_raw_a);
    p_data_b :> data_raw_b;
    adc_data_b[4] = convert(data_raw_b);

    SPI_IDLE;
    t :> ts;
}

void adc_ad7949_triggered(interface ADCInterface server i_adc[2], AD7949Ports &adc_ports,
                                CurrentSensorsConfig &current_sensor_config, chanend c_trig)
{
    timer t;
    unsigned int ts;

    unsigned char ct;
    const unsigned int adc_config_mot     =   0b11110001001001;   /* Motor current (ADC Channel 0), unipolar, referenced to GND */
    const unsigned int adc_config_other[] = { 0b10110001001001,   // Temperature
                                              0b11110101001001,   // ADC Channel 2, unipolar, referenced to GND  voltage and current
                                              0b11111001001001,   // ADC Channel 4, unipolar, referenced to GND
                                              0b11111011001001 }; // ADC Channel 5, unipolar, referenced to GND

    const unsigned int delay = (11*USEC_FAST) / 3; // 3.7 us
    unsigned int adc_data_a[5];
    unsigned int adc_data_b[5];
    unsigned short adc_index = 0;
    int i_calib_a = 0, i_calib_b = 0, i = 0, Icalibrated_a = 0, Icalibrated_b = 0;

    configure_adc_ports(adc_ports.clk, adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b);

    //Calibration
    while (i < ADC_CALIB_POINTS) {
        // get ADC reading

        adc_ad7949_singleshot(adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b, adc_ports.clk,
                                               adc_config_mot,  adc_config_other, delay, t, adc_data_a, adc_data_b, adc_index);

        if (adc_data_a[4]>0 && adc_data_a[4]<16384  &&  adc_data_b[4]>0 && adc_data_b[4]<16384) {
            i_calib_a += adc_data_a[4];
            i_calib_b += adc_data_b[4];
            i++;
            if (i == ADC_CALIB_POINTS) {
                break;
            }
        }
    }

   i_calib_a = (i_calib_a >> Factor);
   i_calib_b = (i_calib_b >> Factor);

    while (1)
    {
#pragma ordered
        select
        {

        case inct_byref(c_trig, ct):
            if (ct == XS1_CT_END)
            {
                t :> ts;
                t when timerafter(ts + 7080) :> ts; // 6200

                adc_ad7949_singleshot( adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b,
                                        adc_ports.clk, adc_config_mot,	adc_config_other, delay, t, adc_data_a,
                                        adc_data_b, adc_index);
            }

            break;

        case i_adc[int i].get_currents() -> {int Ia, int Ib}:

                Ia = current_sensor_config.sign_phase_b * Icalibrated_a;
                Ib = current_sensor_config.sign_phase_c * Icalibrated_b;

                break;

        case i_adc[int i].get_temperature() -> {int out_temp}:

                out_temp = adc_data_a[1];

                break;

        case i_adc[int i].get_external_inputs() -> {int ext_a, int ext_b}:

                ext_a = adc_data_a[3];
                ext_b = adc_data_b[3];

                break;

        case i_adc[int i].helper_amps_to_ticks(float amps) -> int out_ticks:


                if(amps >= current_sensor_config.current_sensor_amplitude)
                     out_ticks = MAX_ADC_VALUE/2; break;
                if(amps <= -current_sensor_config.current_sensor_amplitude)
                    out_ticks = -MAX_ADC_VALUE/2; break;

                out_ticks = (int) amps * (MAX_ADC_VALUE/(2*current_sensor_config.current_sensor_amplitude));

                break;

        case i_adc[int i].helper_ticks_to_amps(int ticks) -> float out_amps:

                if(ticks >= MAX_ADC_VALUE/2)
                    out_amps = current_sensor_config.current_sensor_amplitude; break;
                if(ticks <= -MAX_ADC_VALUE/2)
                    out_amps = -current_sensor_config.current_sensor_amplitude; break;

                out_amps = ticks/(MAX_ADC_VALUE/2.0) * current_sensor_config.current_sensor_amplitude;

                break;
        }

        Icalibrated_a = ((int) adc_data_a[4]) - i_calib_a;
        Icalibrated_b =((int) adc_data_b[4]) - i_calib_b;
    }
}


void adc_ad7949(interface ADCInterface server i_adc[2], AD7949Ports &adc_ports,
                                CurrentSensorsConfig &current_sensor_config)
{
    timer t;
    const unsigned int adc_config_mot     =   0b11110001001001;   /* Motor current (ADC Channel 0), unipolar, referenced to GND */
    const unsigned int adc_config_other[] = { 0b10110001001001,   // Temperature
                                              0b11110101001001,   // ADC Channel 2, unipolar, referenced to GND  voltage and current
                                              0b11111001001001,   // ADC Channel 4, unipolar, referenced to GND
                                              0b11111011001001 }; // ADC Channel 5, unipolar, referenced to GND

    const unsigned int delay = (11*USEC_FAST) / 3; // 3.7 us
    unsigned int adc_data_a[5];
    unsigned int adc_data_b[5];
    unsigned short adc_index = 0;
    int i_calib_a = 0, i_calib_b = 0, i = 0, Icalibrated_a = 0, Icalibrated_b = 0;

    configure_adc_ports(adc_ports.clk, adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b);

    //Calibration
    while (i < ADC_CALIB_POINTS) {
        // get ADC reading

        adc_ad7949_singleshot(adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b, adc_ports.clk,
                                               adc_config_mot,  adc_config_other, delay, t, adc_data_a, adc_data_b, adc_index);

        if (adc_data_a[4]>0 && adc_data_a[4]<16384  &&  adc_data_b[4]>0 && adc_data_b[4]<16384) {
            i_calib_a += adc_data_a[4];
            i_calib_b += adc_data_b[4];
            i++;
            if (i == ADC_CALIB_POINTS) {
                break;
            }
        }
    }

   i_calib_a = (i_calib_a >> Factor);
   i_calib_b = (i_calib_b >> Factor);

    while (1)
    {
#pragma ordered
        select
        {

        case i_adc[int i].get_currents() -> {int Ia, int Ib}:


                //If no trigger exists on the system, we sample on request
                adc_ad7949_singleshot( adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b,
                                        adc_ports.clk, adc_config_mot,  adc_config_other, delay, t, adc_data_a,
                                        adc_data_b, adc_index);

                Icalibrated_a = ((int) adc_data_a[4]) - i_calib_a;
                Icalibrated_b = ((int) adc_data_b[4]) - i_calib_b;

                Ia = current_sensor_config.sign_phase_b * Icalibrated_a;
                Ib = current_sensor_config.sign_phase_c *Icalibrated_b;

                break;

        case i_adc[int i].get_temperature() -> {int out_temp}:

                //If no trigger exists on the system, we sample on request
                adc_index = 1;
                adc_ad7949_singleshot( adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b,
                                        adc_ports.clk, adc_config_mot,  adc_config_other, delay, t, adc_data_a,
                                        adc_data_b, adc_index);

                out_temp = adc_data_a[1];

                break;

        case i_adc[int i].get_external_inputs() -> {int ext_a, int ext_b}:

                //If no trigger exists on the system, we sample on request
                adc_index = 3;
                adc_ad7949_singleshot( adc_ports.sclk_conv_mosib_mosia, adc_ports.data_a, adc_ports.data_b,
                                        adc_ports.clk, adc_config_mot,  adc_config_other, delay, t, adc_data_a,
                                        adc_data_b, adc_index);

                ext_a = adc_data_a[3];
                ext_b = adc_data_b[3];

                break;

        case i_adc[int i].helper_amps_to_ticks(float amps) -> int out_ticks:


                if(amps >= current_sensor_config.current_sensor_amplitude)
                     out_ticks = MAX_ADC_VALUE/2; break;
                if(amps <= -current_sensor_config.current_sensor_amplitude)
                    out_ticks = -MAX_ADC_VALUE/2; break;

                out_ticks = (int) amps * (MAX_ADC_VALUE/(2*current_sensor_config.current_sensor_amplitude));

                break;

        case i_adc[int i].helper_ticks_to_amps(int ticks) -> float out_amps:

                if(ticks >= MAX_ADC_VALUE/2)
                    out_amps = current_sensor_config.current_sensor_amplitude; break;
                if(ticks <= -MAX_ADC_VALUE/2)
                    out_amps = -current_sensor_config.current_sensor_amplitude; break;

                out_amps = ticks/(MAX_ADC_VALUE/2.0) * current_sensor_config.current_sensor_amplitude;

                break;
        }
    }
}


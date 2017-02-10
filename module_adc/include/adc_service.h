/**
 * @file adc_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>
#include <watchdog_service.h>
#include <motor_control_interfaces.h>

/**
 * Max value the ADC can provide.
 */
#define MAX_ADC_VALUE 16383
/**
 * Max allowed value in ADC ticks till overcurrent protection is triggered.
 */
#define OVERCURRENT_IN_ADC_TICKS 12800 //modify the value to match your motor operating conditions. Should be lower than MAX_ADC_VALUE!

#define AD7265_SGL_A1_B1 0b1000
#define AD7265_SGL_A2_B2 0b1001
#define AD7265_SGL_A3_B3 0b1010
#define AD7265_SGL_A4_B4 0b1011
#define AD7265_SGL_A5_B5 0b1100
#define AD7265_SGL_A6_B6 0b1101

#define AD7265_DIFF_A1A2_B1B2  0b0000
#define AD7265_DIFF_A3A4_B3B4  0b0010
#define AD7265_DIFF_A5A6_B5B6  0b0101


#define AD7949_TEMPERATURE          0b10110001001001

#define AD7949_CHANNEL_0            0b11110001001001
#define AD7949_CHANNEL_1            0b11110011001001
#define AD7949_CHANNEL_2            0b11110101001001
#define AD7949_CHANNEL_3            0b11110111001001
#define AD7949_CHANNEL_4            0b11111001001001
#define AD7949_CHANNEL_5            0b11111011001001
#define AD7949_CHANNEL_6            0b11111101001001
#define AD7949_CHANNEL_7            0b11111111001001

#define SINGLE_ENDED                0
#define DIFFERENTIAL                1

/**
 * @brief Operational mode of ADC
 *
 */
typedef enum
{
    NORMAL_MODE = 1,
    STD_MOTOR_CTRL_MODE = 2
} AdcOperationalMode;

typedef enum
{
    AD_7949_TEMPERATURE      =0,
    AD_7949_VMOT_DIV_I_MOT   =1,
    AD_7949_EXT_A0_N_EXT_A1_N=2,
    AD_7949_EXT_A0_P_EXT_A1_P=3
} Ad7949ChannelInputs;

typedef enum
{
    AD_7265_CURRENT_B_C     =0,                     //corresponding to VA1 & VB1 in AD7265 (dc1k board)
    AD_7265_VDC_IDC         =1,                     //corresponding to VA2 & VB2 in AD7265 (dc1k board)
    AD_7265_AI_SIGNAL_1_3   =2,                     //corresponding to VA3 & VB3 in AD7265 (dc1k board)
    AD_7265_AI_SIGNAL_2_4   =3,                     //corresponding to VA4 & VB4 in AD7265 (dc1k board)
    AD_7265_BOARD_TEMP_PHASE_VOLTAGE_B  =4,         //corresponding to VA5 & VB5 in AD7265 (dc1k board)
    AD_7265_PHASE_VOLTAGE_C_PHASE_VOLTAGE_B  =5     //corresponding to VA6 & VB6 in AD7265 (dc1k board)
} Ad7265ChannelInputs;



/**
 * Structure type to define the ports to manage the AD7949 ADC chip.
 */
typedef struct {
     buffered out port:32 ?sclk_conv_mosib_mosia;   /**< [[Nullable]] 4-bit Port for ADC management */
     in buffered port:32 ?data_a;   /**< [[Nullable]] 32-bit buffered ADC data port a */
     in buffered port:32 ?data_b;   /**< [[Nullable]] 32-bit buffered ADC data port b */
     clock ?clk;                    /**< [[Nullable]] Internal XMOS clock. */
}AD7949Ports;

/**
 * Structure type to define the ports to manage the AD7265 ADC chip.
 */
typedef struct {
    in buffered port:32 ?p32_data[2]; /**< [[Nullable]] Array of 32-bit buffered ADC data ports. */
    clock ?xclk;  /**< [[Nullable]] Internal XMOS clock. */
    out port ?p1_serial_clk; /**< [[Nullable]] Port connecting to external ADC serial clock. */
    port ?p1_ready;   /**< [[Nullable]] Port used to as ready signal for p32_adc_data ports and ADC chip. */
    out port ?p4_mux; /**< [[Nullable]] 4-bit Port used to control multiplexor on ADC chip. */
} AD7265Ports;

/**
 * Structure type for current measurement configuration.
 */
typedef struct {
    int sign_phase_b;   /**< Direction in which current on B Phase is measured [-1,1]. */
    int sign_phase_c;   /**< Direction in which current on C Phase is measured [-1,1]. */
    unsigned current_sensor_amplitude;  /**< Max amplitude of current the sensors on your DC board can handle. */
}CurrentSensorsConfig;

/**
 * Structure type for ports and configuration used by the ADC Service .
 */
typedef struct {
    AD7949Ports ad7949_ports;                  /**< Structure containing ports information about AD7949 chip (if applicable) */
    AD7265Ports ad7265_ports;                   /**< Structure containing ports information about AD7265 chip (if applicable) */
    CurrentSensorsConfig current_sensor_config; /**< Configuration about the current measurement */
} ADCPorts;

/**
 * @brief Service providing readings from the ADC chip in your SOMANET device.
 *        Measurements can be sampled on request, or can be triggered over a communication
 *        channel by just providing such channel.
 *
 * @param adc_ports Ports structure defining where to access the ADC chip signals.
 * @param c_trigger [[Nullable]] Channel communication to trigger sampling. If not provided, sampling takes place on request.
 * @param i_adc Array of communication interfaces to handle up to 5 different clients.
 */
void adc_service(ADCPorts &adc_ports, interface ADCInterface server i_adc[2], interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec, int operational_mode);

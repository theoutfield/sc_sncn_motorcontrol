/**
 * @file adc_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>
#include <watchdog_service.h>
#include <motor_control_interfaces.h>

/**
 * @brief Define possible sampling modes in case of using AD7265 chip (SINGLE_ENDED/DIFFERENTIAL)
 */
#define SINGLE_ENDED                0
#define DIFFERENTIAL                1

/**
 * @brief Define proper multiplexer values of AD7265 chip in SINGLE_ENDED sampling mode
 */
#define AD7265_SGL_A1_B1 0b1000
#define AD7265_SGL_A2_B2 0b1001
#define AD7265_SGL_A3_B3 0b1010
#define AD7265_SGL_A4_B4 0b1011
#define AD7265_SGL_A5_B5 0b1100
#define AD7265_SGL_A6_B6 0b1101

/**
 * @brief Define proper multiplexer values of AD7265 chip in FULLY_DIFFERENTIAL sampling mode
 */
#define AD7265_DIFF_A1A2_B1B2  0b0000
#define AD7265_DIFF_A3A4_B3B4  0b0010
#define AD7265_DIFF_A5A6_B5B6  0b0101

/**
 * @brief Define proper configuration register values of AD7949 chip in UNIPOLAR (SINGLE_ENDED) sampling mode
 */
#define AD7949_CHANNEL_0            0b11110001001001
#define AD7949_CHANNEL_1            0b11110011001001
#define AD7949_CHANNEL_2            0b11110101001001
#define AD7949_CHANNEL_3            0b11110111001001
#define AD7949_CHANNEL_4            0b11111001001001
#define AD7949_CHANNEL_5            0b11111011001001
#define AD7949_CHANNEL_6            0b11111101001001
#define AD7949_CHANNEL_7            0b11111111001001

/**
 * @brief Possible selection of analogue input pairs (to be sampled) in case of using AD7949 chip.
 * used internally in AD7949 service.
 */
typedef enum
{
    AD_7949 =0,
    AD_7265 =1
} AdcType;

/**
 * @brief Possible selection of analogue input pairs (to be sampled) in case of using AD7949 chip.
 * used internally in AD7949 service.
 */
typedef enum
{
    AD_7949_IB_IC            =0,
    AD_7949_VMOT_DIV_I_MOT   =1,
    AD_7949_EXT_A0_N_EXT_A1_N=2,
    AD_7949_EXT_A0_P_EXT_A1_P=3
} Ad7949ChannelInputs;


/**
 * Structure type for channel mapping
 */
typedef struct
{
    int current_a;
    int current_b;
    int voltage_dc;
    int current_dc;
    int temperature;
    int voltage_a;
    int voltage_b;
    int voltage_c;
    int analogue_input_1;
    int analogue_input_2;
    int analogue_input_3;
    int analogue_input_4;
    int analogue_input_differential_mode_1;
    int analogue_input_differential_mode_2;
} Ad7265ChannelIndex;

/**
 * @brief Fault Codes for sending to watchdog service
 */
typedef enum {
    WD_OVER_CURRENT_PHASE_A = 1,
    WD_OVER_CURRENT_PHASE_B = 2,
    WD_OVER_CURRENT_PHASE_C = 3,
    WD_UNDER_VOLTAGE = 4,
    WD_OVER_VOLTAGE = 5,
    WD_OVER_TEMPERATURE = 6
} WatchdogFaultCode;

/**
 * Structure type to define the ports to manage the AD7949 ADC chip.
 */
typedef struct
{
     buffered out port:32 ?sclk_conv_mosib_mosia;   /**< [[Nullable]] 4-bit Port for ADC management     */
     in buffered port:32 ?data_a;                   /**< [[Nullable]] 32-bit buffered ADC data port a   */
     in buffered port:32 ?data_b;                   /**< [[Nullable]] 32-bit buffered ADC data port b   */
     clock ?clk;                                    /**< [[Nullable]] Internal XMOS clock.              */
}AD7949Ports;

/**
 * Structure type to define the ports to manage the AD7265 ADC chip.
 */
typedef struct
{
    in buffered port:32 ?p32_data[2];   /**< [[Nullable]] Array of 32-bit buffered ADC data ports. */
    clock ?xclk;                        /**< [[Nullable]] Internal XMOS clock. */
    out port ?p1_serial_clk;            /**< [[Nullable]] Port connecting to external ADC serial clock. */
    port ?p1_ready;                     /**< [[Nullable]] Port used to as ready signal for p32_adc_data ports and ADC chip. */
    out port ?p4_mux;                   /**< [[Nullable]] 4-bit Port used to control multiplexor on ADC chip. */
    Ad7265ChannelIndex ad7265_channel_index;
} AD7265Ports;

/**
 * Structure type for current measurement configuration.
 */
typedef struct
{
    int sign_phase_b;                   /**< Direction in which current on B Phase is measured [-1,1]. */
    int sign_phase_c;                   /**< Direction in which current on C Phase is measured [-1,1]. */
    unsigned current_sensor_amplitude;  /**< Max amplitude of current the sensors that your DC board can handle. */
}CurrentSensorsConfig;


/**
 * Structure type for ports and configuration used by the ADC Service .
 */
typedef struct
{
    AD7949Ports ad7949_ports;                   /**< Structure containing ports information about AD7949 chip (if applicable) */
    AD7265Ports ad7265_ports;                   /**< Structure containing ports information about AD7265 chip (if applicable) */
    CurrentSensorsConfig current_sensor_config; /**< Configuration about the current measurement */
} ADCPorts;

/**
 * @brief Service providing readings from the ADC chip in your SOMANET device.
 * Measurements can be sampled on requests through i_adc interfaces.
 *
 * @param adc_ports             Ports structure defining where to access the ADC chip signals.
 * @param i_adc[2]              Array of communication interfaces to handle up to 2 different clients.
 * @param i_watchdog            Interface to communicate with watchdog service
 * @param ifm_tile_usec         Reference clock frequency of IFM tile (in MHz)
 * @param operational_mode      Integer type to select between SINGLE_ENDED/FULLY_DIFFERENTIAL modes
 *
 * @return void
 */
void adc_service(ADCPorts &adc_ports, interface ADCInterface server i_adc[2], interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec, int operational_mode);

/**
 * @file hall_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>

#define MAX_ADC_VALUE 16383
/**
 * @brief Lorem ipsum...
 */
interface ADCInterface{
   // void calibrate();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     */
    {int, int, int, int, int, int, int, int} get_all();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     */
    {int, int} get_currents();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_temperature();

    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     */
    {int, int} get_external_inputs();

    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     */
    int helper_amps_to_ticks(float amps);

    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     * @return Lorem ipsum...
     */
    float helper_ticks_to_amps(int ticks);
};

/**
 * Lorem ipsum...
 */
typedef struct {
     buffered out port:32 ?sclk_conv_mosib_mosia;
     in buffered port:32 ?data_a;
     in buffered port:32 ?data_b;
     clock ?clk;
}AD7949Ports;

/**
 * Lorem ipsum...
 */
typedef struct {
    in buffered port:32 ?p32_data[2]; /**< Array of 32-bit buffered ADC data ports */
    clock ?xclk;  /**< Internal XMOS clock */
    out port ?p1_serial_clk; /**< 1-bit port connecting to external ADC serial clock */
    port ?p1_ready;   /**< 1-bit port used to as ready signal for p32_adc_data ports and ADC chip */
    out port ?p4_mux; /**< 4-bit port used to control multiplexor on ADC chip */
} AD7265Ports;

/**
 * Lorem ipsum...
 */
typedef struct {
    int sign_phase_b;
    int sign_phase_c;
    unsigned current_sensor_amplitude;
}CurrentSensorsConfig;

/**
 * Lorem ipsum...
 */
typedef struct {
    AD7949Ports ad7949_ports;
    AD7265Ports ad7265_ports;
    CurrentSensorsConfig current_sensor_config;
} ADCPorts;

/**
 * @brief Lorem ipsum...
 *
 * @param adc_ports Lorem ipsum...
 * @param c_trigger Lorem ipsum...
 * @param adc_interface[3] Lorem ipsum...
 */
void adc_service(ADCPorts &adc_ports, chanend ?c_trigger, interface ADCInterface server adc_interface[2]);

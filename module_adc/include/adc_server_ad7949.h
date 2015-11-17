/**
 * @file adc_server_ad7949.h
 * @brief ADC Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>
#include <xclib.h>
#include <adc.h>

/*
interface AD7949Interface{
    void calibrate();
    {int, int, int, int, int, int, int, int} get_all();
    {int, int} get_currents();
    {int, int} get_external_inputs();
};
*/
/**
 * @brief Non triggered ADC server
 *
 *
 * This server should be used if the somanet node is not used for motor
 * drive/control. This is the interface to AD7949 ADC devices. It controls
 * two devices so that two channels can be sampled simultaneously.
 *
 * @param c_adc channel to receive ADC output
 * @param clk clock for the ADC device serial port
 * @param p_sclk_conv_mosib_mosia 4-bit port for ADC control interface
 * @param p_data_a 1-bit port for ADC data channel 0
 * @param p_data_b 1-bit port for ADC data channel 1
 *
 */
void adc_ad7949(  interface ADCInterface server adc_interface,
                 AD7949Ports &adc_ports );

/**
 * @brief Triggered ADC server
 *
 * This server should be used if the somanet node is used for motor
 * drive/control. This is the interface to AD7949 ADC devices.
 * It controls two devices so that two channels can be sampled
 * simultaneously.
 *
 * @param	c_adc channel to receive ADC output
 * @param	c_trig channel to trigger adc from the PWM modules
 *
 * @param clk clock for the ADC device serial port
 *
 * @param p_sclk_conv_mosib_mosia 4-bit port for ADC control interface
 * @param p_data_a 1-bit port for ADC data channel 0
 * @param p_data_b 1-bit port for ADC data channel 1
 *
 */
void adc_ad7949_triggered( interface ADCInterface server adc_interface,
                            AD7949Ports &adc_ports,
                           chanend c_trig);

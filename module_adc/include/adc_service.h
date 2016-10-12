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
void adc_service(ADCPorts &adc_ports, chanend ?c_trigger, interface ADCInterface server i_adc[2], interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec);

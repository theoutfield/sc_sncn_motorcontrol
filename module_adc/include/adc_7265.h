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

#pragma once

#include <xs1.h>
#include <xclib.h>
#include <assert.h>
#include <adc_service.h>

/*  The AD7265 data-sheet refers to the following signals:-
 *      SCLK:           Serial Clock frequency (can be configured to between  4..16 MHz.)
 *      CS#:            Chip Select. Ready signal (Falling edge starts sample conversion)
 *      A[0..2]:        Multiplexer Select. Selects inputs to be sampled.
 *      SGL/DIFF:       Selects between Single-ended/Differential mode.
 *      RANGE:          Selects between 0..Vref and 0..2xVref.
 *      Vdrive:         Max. analogue voltage corresponding to Max. 12-bit sample (Hardwired to 3.3V)
 *      REF_SELECT: Selects internal/external ref. (Hardwired to 2.5V internal)
 *
 *  The S/W application needs to set A[0..2].
 *  In differential mode (SGl/DIFF = 0):
 *      A[0] selects between Fully/Pseudo Differential. We choose A[0] = 0 for Fully Differential.
 *      A[1,2] is dependant on the motor identifier, and selects between Motor ports V1/V3/V5
 *
 *  The AD7265 returns a sample with 12 active bits of data.
 *  For our configuration (SGL/DIFF=0, A[0]=0, RANGE=0), these bit are in 2's compliment format.
 *
 *  There can also be padding bits (of value zero) both before and after the active bits.
 *  For this application 14, 15, or 16-bit samples can be used.
 *  If timings are set up correctly, the padding bits are expected to be as follows:-
 *
 *  Total-Sample-Size   Pre-Pad(MSB)   Post-Pad(LSB)
 *  -----------------  --------------  -------------
 *      14                 2               0
 *      15                 2               1
 *      16                 2               2
 */

/**
 * @brief Define Number of pre-padding bits before Most-Significant active bit of sample 0..2
 */
#define ADC_PRE_PAD_BITS 2

/**
 * @brief Define Number of active bits in ADC Sample
 */
#define ADC_ACTIVE_BITS 12

/**
 * @brief Define Number of post-padding bits after Least-Significant active bit of sample 0..2
 */
#define ADC_POST_PAD_BITS 0

/**
 * @brief Define Minimum Number of bits to transmit in ADC sample (including pre-padding bits)
 */
#define ADC_MIN_BITS (ADC_ACTIVE_BITS + ADC_PRE_PAD_BITS)

/**
 * @brief Define Difference between Word16 and active bits
 */
#define ADC_DIFF_BITS (WORD16_BITS - ADC_ACTIVE_BITS)

/**
 * @brief Define 14..16 Total Number of bits to in ADC sample (including post-padding bits)
 */
#define ADC_TOTAL_BITS (ADC_MIN_BITS + ADC_POST_PAD_BITS)

/**
 * @brief Define 4..2 Number of bits to shift to get 16-bit word alignment
 */
#define ADC_SHIFT_BITS (ADC_DIFF_BITS - ADC_POST_PAD_BITS)

/**
 * @brief Define Mask for 12 active bits in MS bits of 16-bit word
 */
#define ADC_MASK 0x0FFF

/**
 * @brief Define ADC Serial Clock frequency (in MHz)
 *
 * The AD7265 clock frequency (SCLK) can be configured to between  4..16 MHz.
 * In order to trigger capture from the ADC digital ouput on a rising edge,
 * the SCLK frequency must be less than 13.7 MHz.
 * Considering all above constraints. We set the ADC frequency to 8 MHz
 */
#define ADC_SCLK_MHZ 8


 /**
  * @brief Demo service to show how AD7265 can be used.
  *
  * @param adc_ports             Structure type to manage the AD7265 ADC chip.
  * @param iADC[2]               Interface to communicate with clients and send the measured values
  *
  * @return void
  */
 void adc_ad7265_service_demo(
         AD7265Ports &adc_ports,
         interface ADCInterface server iADC[2]);

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
void adc_ad7265(
        interface ADCInterface server iADC[2],
        AD7265Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog, int operational_mode, int tile_usec);



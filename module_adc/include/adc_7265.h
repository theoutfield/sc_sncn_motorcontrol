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
#include <xclib.h> // NB Contains bitrev()
#include <assert.h>
#include <print.h>
#include <adc_service.h>

#define ADC_CALIB_POINTS 64
#define Factor 6

#define AD7265_MUX_DEFAULT_CONFIG 0b1000 // nDIFF|A2|A1|A0

#define ADC_FIXED_CHANNEL_OPERATION 1 // the channels will be set directly inside adc server
                                      // adc channels can not be modified by server inputs

interface ADC{
    {int, int} get_adc_measurements(unsigned char port_id, unsigned char config);
};

/*  The AD7265 data-sheet refers to the following signals:-
 *      SCLK:               Serial Clock frequency (can be configured to between  4..16 MHz.)
 *      CS#:                Chip Select. Ready signal (Falling edge starts sample conversion)
 *      A[0..2]:        Multiplexer Select. Selects inputs to be sampled.
 *      SGL/DIFF:       Selects between Single-ended/Differential mode.
 *      RANGE:          Selects between 0..Vref and 0..2xVref.
 *      Vdrive:         Max. analogue voltage corresponding to Max. 12-bit sample (Hardwired to 3.3V)
 *      REF_SELECT: Selects internal/external ref. (Hardwired to 2.5V internal)
 *
 *  The Motor-Control Board has been hardwired for differential mode.
 *  There are 2 jumper settings for controlling the following signals:-
 *      SGL/DIFF:       Should be set to 0 for Differential mode.
 *      RANGE:          Should be set to 0 for 0..Vref range.
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

#define ADC_PRE_PAD_BITS 2 // 0..2 No. of pre-padding bits before Most-Significant active bit of sample
#define ADC_ACTIVE_BITS 12 // No. of active bits in ADC Sample
#define ADC_POST_PAD_BITS 0 // 0..2 No. of post-padding bits after Least-Significant active bit of sample
#define ADC_MIN_BITS (ADC_ACTIVE_BITS + ADC_PRE_PAD_BITS) // Minimum No. of bits to transmit in ADC sample (including pre-padding bits)

/* Temperature */
#define ADC_VALUE_0_DEGREES 1640     //0.5V
#define ADC_TEMP_ERROR      1120
#define ADC_VALUE_PER_DEGREE 32    //10mV/deg

/** Define Bits in Byte */
#define BITS_IN_BYTE 8
#define WORD16_BITS (sizeof(short) * BITS_IN_BYTE) // No. of bits in 16-bit word

#define ADC_DIFF_BITS (WORD16_BITS - ADC_ACTIVE_BITS) //4 Difference between Word16 and active bits
#define ADC_TOTAL_BITS (ADC_MIN_BITS + ADC_POST_PAD_BITS) //14..16 Total No. of bits to in ADC sample (including post-padding bits)

#define ADC_SHIFT_BITS (ADC_DIFF_BITS - ADC_POST_PAD_BITS) //4..2 No. of bits to shift to get 16-bit word alignment
#define ADC_MASK 0x0FFF // Mask for 12 active bits in MS bits of 16-bit word

/*  The AD7265 clock frequency (SCLK) can be configured to between  4..16 MHz.
 *  The PWM requires a 16-bit sample every 61 KHz, this translates to a minimum ADC frequency of 977 KHz.
 *  In order to trigger capture from the ADC digital ouput on a rising edge,
 *  the SCLK frequency must be less than 13.7 MHz.
 *  Considering all above constraints. We set the ADC frequency to 8 MHz
 */
#define ADC_SCLK_MHZ 8 // ADC Serial Clock frequency (in MHz)

/* ADC_TRIGGER_DELAY needs to be tuned to move the ADC trigger point into the centre of the PWM 'OFF' period.
 * This value is related to the PWM_MAX_VALUE (in module_pwm_foc) and is independent of the Reference Frequency
 */
#define ADC_TRIGGER_CORR 128 // Timing correction
#define ADC_TRIGGER_DELAY (QUART_PWM_MAX - ADC_TRIGGER_CORR) // MB~ Re-tune

void adc_ad7256(interface ADCInterface server iADC[2], AD7265Ports &adc_ports,
                    CurrentSensorsConfig &current_sensor_config, interface WatchdogInterface client ?i_watchdog);
void adc_ad7256_fixed_channel(interface ADCInterface server iADC[2], AD7265Ports &adc_ports,
                    CurrentSensorsConfig &current_sensor_config, interface WatchdogInterface client ?i_watchdog);
void adc_ad7256_triggered(interface ADCInterface server iADC[2], AD7265Ports &adc_ports,
                    CurrentSensorsConfig &current_sensor_config, chanend c_trig, interface WatchdogInterface client ?i_watchdog);

int statusTemperature_adc2degrees(int adcValue);

/*****************************************************************************/

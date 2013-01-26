#pragma once

/****************************
 * Ifm Drive BLDC 100 Rev A
 ****************************/

/* RGB LED and green LED */
extern out port p_ifm_leds; /* BlueGreenRed_Green */

/* Buttons */
extern in port p_ifm_btn1; /* Button 1 */
/* Button 2 shares port with encoder (see below) */

/* A4935 control ports */
extern out port p_ifm_coastn;
extern out port p_ifm_esf_rstn_pwml_pwmh;
extern     port p_ifm_ff1;
extern     port p_ifm_ff2;

/* ADC */
extern          out port p_ifm_adc_clk;
extern buffered out port:32 p_ifm_adc_conv;
extern in buffered  port:32 p_ifm_adc_miso;
extern in           port p_ifm_dummy_port;

/* BLDC Motor */
extern           in  port p_ifm_hall;
extern  buffered out port:32 p_ifm_motor_hi[3]; /* = { Hi A, Hi B, Hi C } */
//extern  out port p_motor_pwm_hi[3]; /* = { Hi A, Hi B, Hi C } */
extern  buffered out port:32 p_ifm_motor_lo[3]; /* = { Hi A, Hi B, Hi C } */
//extern           out port p_ifm_motor_lo[3]; /* = { Lo A, Lo B, Lo C } */


extern           in port p_ifm_encoder_btn2; /* Encoder and Button 2 */

/* Expansion Port */
extern out port p_ifm_scl; /* I2C SCL */
extern     port p_ifm_sda; /* I2C SDA */

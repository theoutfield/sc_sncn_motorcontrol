#pragma once

/****************************
 * Ifm Drive BLDC 100 Rev A
 ****************************/

/* ADC */
extern in buffered  port:32 p_ifm_adc_misoa;
extern in buffered  port:32 p_ifm_adc_misob;
extern buffered out port:32 p_ifm_adc_sclk_conv_mosib_mosia;


/* A4935 control ports */
extern out port p_ifm_coastn;
extern out port p_ifm_esf_rstn_pwml_pwmh;
extern     port p_ifm_ff1;
extern     port p_ifm_ff2;

/* BLDC Motor, default port mode */
extern           in  port p_ifm_hall;
extern           in  port p_ifm_encoder;
extern           in  port p_ifm_dummy_port; /* needed for PWM ADC-trigger timing */
extern  buffered out port:32 p_ifm_motor_hi[3]; /* = { Hi A, Hi B, Hi C } */
extern  buffered out port:32 p_ifm_motor_lo[3]; /* = { Hi A, Hi B, Hi C } */
/* BLDC Motor, alternative declaration for low-side PWM */
//extern  out port p_motor_pwm_hi[3]; /* = { Hi A, Hi B, Hi C } */
//extern           out port p_ifm_motor_lo[3]; /* = { Lo A, Lo B, Lo C } */

/* LEDs, Watchdog */
extern out port p_ifm_shared_leds_wden; /* BlueGreenRed_WDEN */
extern out port p_ifm_wd_tick;

/* Expansion Port */
extern port p_ifm_ext_d0;
extern port p_ifm_ext_d1;
extern port p_ifm_ext_d2;
extern port p_ifm_ext_d3;


#pragma once

#include <xs1.h>
#include <platform.h>

/**********************
 * Ifm Drive BLDC 100
 **********************/

/* RGB LED and green LED */
on stdcore[3]: out port p_ifm_leds = XS1_PORT_4B; /* BlueGreenRed_Green */

/* Buttons */
on stdcore[3]: in port p_ifm_btn1 = XS1_PORT_1J; /* Button 1 */
/* Button 2 shares port with encoder (see below) */

/* A4935 control ports */
on stdcore[3]: out port p_ifm_coastn = XS1_PORT_1G;
on stdcore[3]: out port p_ifm_esf_rstn_pwml_pwmh = XS1_PORT_4F;
on stdcore[3]: port p_ifm_ff1 = XS1_PORT_1H;
on stdcore[3]: port p_ifm_ff2 = XS1_PORT_1I;

/* ADC */
on stdcore[3]:          out port p_ifm_adc_clk = XS1_PORT_1B;
on stdcore[3]: buffered out port:32 p_ifm_adc_conv = XS1_PORT_1C;
on stdcore[3]: in buffered  port:32 p_ifm_adc_miso = XS1_PORT_1D;
on stdcore[3]: in           port p_ifm_dummy_port = XS1_PORT_16A; /* any unused port */

/* BLDC Motor */
on stdcore[3]:          in  port p_ifm_hall = XS1_PORT_4E;
on stdcore[3]: buffered out port:32 p_ifm_motor_hi[3] = { XS1_PORT_1K,   /* Hi A */
							XS1_PORT_1O,   /* Hi B */
							XS1_PORT_1M }; /* Hi C */
//on stdcore[3]: out port p_ifm_motor_hi[3] = { XS1_PORT_1K,   /* Hi A */
//					      XS1_PORT_1O,   /* Hi B */
//					      XS1_PORT_1M }; /* Hi C */
on stdcore[3]: buffered out port:32 p_ifm_motor_lo[3] = { XS1_PORT_1L,   /* Lo A */
							  XS1_PORT_1P,   /* Lo B */
							  XS1_PORT_1N }; /* Lo C */
//on stdcore[3]: out port p_ifm_motor_lo[3] = { XS1_PORT_1L,   /* Lo A */
//					      XS1_PORT_1P,   /* Lo B */
//					      XS1_PORT_1N }; /* Lo C */
on stdcore[3]: in port p_ifm_encoder_btn2 = XS1_PORT_4A; /* Encoder and Button 2 */

/* Expansion Port */
on stdcore[3]: out port p_ifm_scl = XS1_PORT_1E; /* I2C SCL */
on stdcore[3]:     port p_ifm_sda = XS1_PORT_1A; /* I2C SDA */

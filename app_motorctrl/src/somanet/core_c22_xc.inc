#pragma once

#include <xs1.h>
#include <platform.h>

/************
 * Core C22
 ************/

/* SPI Flash */
on stdcore[0]: in  port p_core_spiflash_miso = XS1_PORT_1A;
on stdcore[0]: out port p_core_spiflash_ss   = XS1_PORT_1B;
on stdcore[0]: out port p_core_spiflash_clk  = XS1_PORT_1C;
on stdcore[0]: out port p_core_spiflash_mosi = XS1_PORT_1D;

/* RGB LED */
on stdcore[1]: out port p_core_leds[3] = { XS1_PORT_1M,   /* Red */
					   XS1_PORT_1L,   /* Green */
					   XS1_PORT_1K }; /* Blue */

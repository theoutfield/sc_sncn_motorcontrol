#pragma once

/************
 * Core C22
 ************/

/* SPI Flash */
extern in  port p_core_spiflash_miso;
extern out port p_core_spiflash_ss;
extern out port p_core_spiflash_clk;
extern out port p_core_spiflash_mosi;

/* RGB LED */
extern out port p_core_leds[3]; /* = { Red, Green, Blue } */


/**
 * \file a4935.h
 * \brief Driver header file for motor
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#pragma once


#include <refclk.h>

// Bit mapping of 4-bit A4935 config port
#define A4935_BIT_ESF  0x8
#define A4935_BIT_RSTN 0x4
#define A4935_BIT_PWML 0x2
#define A4935_BIT_PWMH 0x1

#define A4935_AFTER_RESET_DELAY (200 * MSEC_FAST/*TICKS_MS*/) // 200ms

/* e.g. a4935_init(p_mgmt, p_coast, A4935_BIT_PWMH | A4935_BIT_PWML); */
void a4935_init(int configuration, out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn);


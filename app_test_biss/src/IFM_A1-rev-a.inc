/***************************
 * IFM
 * Rev. A
 ***************************/

#pragma once

#include <xs1.h>
#include <platform.h>

//#define IFM_RESOLUTION 740


on tile[IFM_TILE]: port p_ifm_encoder_ch1 = XS1_PORT_4A; /* Encoder CH1  */
on tile[IFM_TILE]: port p_ifm_encoder_ch2 = XS1_PORT_4B; /* Encoder CH2  */
on tile[IFM_TILE]: out port qei_q_ch1 = XS1_PORT_1D;     /* Q_CH1 output */
on tile[IFM_TILE]: clock clk_biss = XS1_CLKBLK_1 ;

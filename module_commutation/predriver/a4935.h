#pragma once

#include <xccompat.h>
#include "refclk.h"
//#include "../dsc_config.h"

/* how many ns is a clk tick? */
//#define CLK_NS_PER_TICK  (1000000000U / XS1_TIMER_HZ)

/* define delays based on clk ticks */
//#define TICKS_US     (1000U / CLK_NS_PER_TICK)
//#define TICKS_MS     (1000U * TICKS_US)
//#define TICKS_SEC    (1000U * TICKS_MS)

// Bit mapping of 4-bit A4935 config port
#define A4935_BIT_ESF  0x8
#define A4935_BIT_RSTN 0x4
#define A4935_BIT_PWML 0x2
#define A4935_BIT_PWMH 0x1

#define A4935_AFTER_RESET_DELAY (200 * MSEC_FAST/*TICKS_MS*/) // 200ms

/* #ifdef __XC__ */
/* typedef struct a4935_interface_t { */
/*   out port coastn; */
/*   out port esf_rstn_pwml_pwmh; */
/*   port ff1; */
/*   port ff2; */
/* } a4935_interface_t; */
/* #endif */

//void a4935_init(port out p_esf_rstn_pwml_pwmh, port out p_coastn, unsigned configuration);
/* e.g. a4935_init(p_mgmt, p_coast, A4935_BIT_PWMH | A4935_BIT_PWML); */
void a4935_init(unsigned configuration);


enum a4935_fault_states { A4935_NO_FAULT,
		          A4935_SHORT_FAULT,
			  A4935_OVERTEMP_FAULT,
			  A4935_UNDERVOLTAGE_FAULT };

select a4935_check_fault_select(void);


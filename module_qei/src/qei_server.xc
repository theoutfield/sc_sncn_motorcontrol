/*
 * File:    qei_server.xc
 *
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
 *
 */

#include <xs1.h>
#include "qei_server.h"
#include "qei_commands.h"
#include <stdio.h>

// This is the loop time for 4000RPM on a 1024 count QEI
#pragma xta command "analyze loop qei_main_loop"
#pragma xta command "set required - 14.64 us"

// Order is 00 -> 10 -> 11 -> 01
// Bit 3 = Index
static const unsigned char lookup[16][4] = {
		{ 5, 4, 6, 5 }, // 00 00
		{ 6, 5, 5, 4 }, // 00 01
		{ 4, 5, 5, 6 }, // 00 10
		{ 5, 6, 4, 5 }, // 00 11
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx

		{ 5, 4, 6, 5 }, // 10 00
		{ 6, 5, 5, 4 }, // 10 01
		{ 4, 5, 5, 6 }, // 10 10
		{ 5, 6, 4, 5 }, // 10 11
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }  // 11 xx
};

#if 0
// Order is 00 -> 01 -> 11 -> 10
static const unsigned char lookup[16][4] = {
		{ 5, 6, 4, 5 }, // 00 00
		{ 0, 0, 0, 0 }, // 0x x1
		{ 4, 5, 5, 6 }, // 00 10
		{ 0, 0, 0, 0 }, // 0x x1
		{ 6, 5, 5, 4 }, // 01 00
		{ 0, 0, 0, 0 }, // 0x x1
		{ 5, 4, 6, 5 }, // 01 10
		{ 0, 0, 0, 0 }, // 0x x1

		{ 5, 6, 4, 5 }, // 10 00
		{ 0, 0, 0, 0 }, // 1x x1
		{ 4, 5, 5, 6 }, // 10 10
		{ 0, 0, 0, 0 }, // 1x x1
		{ 6, 5, 5, 4 }, // 11 00
		{ 0, 0, 0, 0 }, // 1x x1
		{ 5, 4, 6, 5 }, // 11 10
		{ 0, 0, 0, 0 }  // 1x x1
};
#endif

#pragma unsafe arrays
void do_qei ( streaming chanend c_qei, port in pQEI )
{
	unsigned pos = 0, v, ts1, ts2, ok=0, old_pins=0, new_pins;
	timer t;

	pQEI :> new_pins;
	t :> ts1;

	while (1) {
#pragma xta endpoint "qei_main_loop"
		select {
			case pQEI when pinsneq(new_pins) :> new_pins :
			{
			  if ((new_pins & 0x3) != old_pins) {
			    //if (((new_pins >> 1) & 0x6) != old_pins) {
			    ts2 = ts1;
			    t :> ts1;
			  }
				v = lookup[new_pins][old_pins];
				if (!v) {
					// v == 0 when 1 is received on "index"-line
					// (cf. empty lines in lut above)
					pos = 0;
					ok = 1;
				} else {
				  // v=high_word, pos=low_word
				  //   lmul: 1*pos + v + -5
				  // including values from lut, this gives 1*pos + {-1 / 0 / +1}
				  { v, pos } = lmul(1, pos, v, -5);
				}
				old_pins = new_pins & 0x3;
				//old_pins = (new_pins >> 1) & 0x6;
			}
			break;
			case c_qei :> int :
			{
				c_qei <: pos;
				c_qei <: ts1;
				c_qei <: ts2;
				c_qei <: ok;
			}
			break;
		}
	}
}





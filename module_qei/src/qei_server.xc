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

#pragma unsafe arrays
void run_qei ( chanend c_qei, port in pQEI )
{
	unsigned pos = 0, v, ts1, ts2, ok=0, old_pins=0, new_pins;
	timer t;

	int cmd;
	int c_pos = 0, prev = 0, count = 0, first = 1;
	int max_count = 26 * 4000;
	int difference = 0, dirn = 0;
	int QEI_COUNT_MAX = 4096;

	pQEI :> new_pins;
	t :> ts1;

	while (1) {
	#pragma ordered
		select {
			case pQEI when pinsneq(new_pins) :> new_pins :
				{
				  if ((new_pins & 0x3) != old_pins) {
					ts2 = ts1;
					t :> ts1;
				  }
					v = lookup[new_pins][old_pins];
					if (!v) {
						pos = 0;
						ok = 1;
					} else {
					  { v, pos } = lmul(1, pos, v, -5);
					}
					old_pins = new_pins & 0x3;
				}
				break;
			case c_qei :> cmd :
				if(cmd == 1)
				{
					slave
					{
						c_qei <: pos;
						c_qei <: ts1;
						c_qei <: ts2;
						c_qei <: ok;
					}
				}
				else if(cmd == 2)
				{
					slave
					{
						c_qei <: count;
						c_qei <: dirn;
					}
				}
				break;

			default:
				if(first == 1)
				{
					prev = pos & (QEI_COUNT_MAX-1);
					first = 0;
				}
				c_pos =  pos & (QEI_COUNT_MAX-1);
				if(prev != c_pos )
				{
					difference = c_pos - prev;
					if( difference > 3000)
					{
						count = count + 1;
						dirn = 1;
					}
					else if(difference < -3000)
					{
						count = count - 1;
						dirn = -1;
					}
					else if( difference < 10 && difference >0)
					{
						count = count - difference;
						dirn = -1;
					}
					else if( difference < 0 && difference > -10)
					{
						count = count - difference;
						dirn = 1;
					}
					prev = c_pos;
				}
				if(count >= max_count || count <= -max_count)
				{
					count=0;
				}
				break;
		}
	}
}





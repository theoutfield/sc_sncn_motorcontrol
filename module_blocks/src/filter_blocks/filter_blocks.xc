
/**
 * \file filter_blocks.xc
 * \brief Moving Average Filter Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "filter_blocks.h"

/* initialization for fixed length digital filter (moving average filter)*/
void init_filter(int filter_buffer[], int &index, int filter_length)
{
	int i;
	for(i=0; i<filter_length; i++)
	{
		filter_buffer[i] = 0;
	}
	index = 0;
}

/* fixed length digital filter (moving average filter)*/
int filter(int filter_buffer[], int &index, int filter_length, int input)
{
	int i, j = 0, mod, filter_output =0;
	filter_buffer[index] = input;
	index = (index+1)%(filter_length);

	for(i=0; i<filter_length; i++)
	{
		mod = (index - 1 - j)%filter_length;
		if(mod<0)
			mod = filter_length + mod;
		filter_output += filter_buffer[mod];
		j++;
	}
	filter_output = filter_output/ filter_length;
	return filter_output;
}


int _modified_internal_filter(int filter_buffer[], int &index, int filter_length, int input)
{
	int i, j = 0, mod, filter_output =0;
	filter_buffer[index] = input;
	index = (index+1)%(filter_length);

	for(i=0; i<filter_length; i++)
	{
		mod = (index - 1 - j)%filter_length;
		if(mod<0)
			mod = filter_length + mod;
		filter_output += filter_buffer[mod];
		j++;
	}
	return filter_output;
}

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

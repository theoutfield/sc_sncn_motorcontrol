/**
 * Module:  module_dsc_adc
 * Version: 1v0alpha2
 * Build:   2a548667d36ce36c64c58f05b5390ec71cb253fa
 * File:    adc_client.xc
 * Modified by : Srikanth
 * Last Modified on : 31-May-2011
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/                                   
#include <xs1.h>
#include<print.h>
#define ADC_CALIB_POINTS	1024

int Ia_calib = 0, Ib_calib = 0, Ic_calib = 0;

void do_adc_calibration_ltc1408( chanend c_adc )
{
	unsigned a,b,c;
	int i = 0;
	while ( i < ADC_CALIB_POINTS)
	{
		/* get ADC reading */
		c_adc <: 3;
		slave
		{
			c_adc :> a;
			c_adc :> b;
			c_adc :> c;
		}
		if(a>0 && a<16384 &&  b>0 && b<16384 && c>0 && c<16384 )
		{
			Ia_calib += a;
			Ib_calib += b;
			Ic_calib += c;
			i++;
			if(i==1024)
				break;
		}
	}
	    Ia_calib = (Ia_calib >> 10);
		Ib_calib = (Ib_calib >> 10);
		Ic_calib = (Ic_calib >> 10);
		printstr("ias ");   printint(Ia_calib); printstr("\n");
		printstr("ibs ");   printint(Ib_calib); printstr("\n");
		printstr("ics ");   printint(Ic_calib); printstr("\n\n");
}

{unsigned, unsigned, unsigned} get_adc_vals_ltc1408( chanend c_adc )
{
	unsigned a, b, c;

	c_adc <: 0;

	slave
	{
		c_adc :> a;
		c_adc :> b;
		c_adc :> c;
	}

	return {a,b,c};
}

{int, int, int} get_adc_calibrated_current_ltc1408( chanend c_adc )
{
	unsigned a, b, c;
	int Ia, Ib, Ic;

	/* request and then receive adc data */
	c_adc <: 3;

	slave
	{
		c_adc :> a;
		c_adc :> b;
		c_adc :> c;
	}
	/* apply calibration offset */

	Ia = a - Ia_calib;
	Ib = b - Ib_calib;
	Ic = c - Ic_calib;

	return {Ia, Ib, Ic};
}





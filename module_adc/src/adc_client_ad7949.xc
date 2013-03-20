
/**
 * \file adc_client_ad7949.xc
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include <adc_ad7949.h>

#define ADC_CALIB_POINTS 64
#define Factor 6

calib_data I_calib;

void do_adc_calibration_ad7949( chanend c_adc )
{
	unsigned a, b;
	int i = 0; I_calib.Ia_calib = 0; I_calib.Ib_calib = 0;
	while ( i < ADC_CALIB_POINTS)
	{
		/* get ADC reading */
		c_adc <: 0;

		slave {
			c_adc :> a;
			c_adc :> b;
		}
		if(a>0 && a<16384 &&  b>0 && b<16384)
		{
			I_calib.Ia_calib += a;
			I_calib.Ib_calib += b;
			i++;
			if(i == ADC_CALIB_POINTS)
				break;
		}
	}
	I_calib.Ia_calib = (I_calib.Ia_calib >> Factor);
	I_calib.Ib_calib = (I_calib.Ib_calib >> Factor);

	/*printstr("ia_calibration ");   printint(I_calib.Ia_calib); printstr("\n");
	printstr("ib_calibration ");   printint(I_calib.Ib_calib); printstr("\n");*/

}


{unsigned, unsigned} get_adc_vals_raw_ad7949( chanend c_adc )
{
  unsigned a, b;

  c_adc <: 0;

  slave {
    c_adc :> a;
    c_adc :> b;
  }

  return {a, b};
}


{int, int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc )
{
  //const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

  unsigned a, b;
  int Ia, Ib;

  c_adc <: 0;

  slave {
    c_adc :> a;
    c_adc :> b;
  }

  Ia = 	(int) a - I_calib.Ia_calib;		//-(Ib + Ic);		/* a + b + c = 0 */  //Ib = (int) b - I_calib.Ia_calib;  //Ic = (int) c - I_calib.Ib_calib;
  Ib = 	(int) b - I_calib.Ib_calib;

  return { Ia, Ib };
}

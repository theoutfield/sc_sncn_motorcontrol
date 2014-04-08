
/**
 * \file adc_client_ad7949.xc
 * \brief ADC Client
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
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

#include <adc_common.h>

//calib_data I_calib;
#define ADC_CURRENT_REQ	1
#define ADC_ALL_REQ	2
#define ADC_CALIB_POINTS 64
#define ADC_EXTERNAL_POT 3
#define Factor 6


{int, int, int, int, int, int, int, int} get_adc_all_ad7949(chanend c_adc)
{
	//const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

	int current_a, current_b;
	int adc_a1, adc_Temperature1, adc_VoltageSupply, ExternalPot1;
	int adc_b1, adc_Temperature2, adc_Dummy, ExternalPot2;


	c_adc <: ADC_ALL_REQ;
	slave
	{
		c_adc :> current_a;
		c_adc :> current_b;
		c_adc :> adc_a1;
		c_adc :> adc_Temperature1;
		c_adc :> adc_VoltageSupply;
		c_adc :> ExternalPot1;
		c_adc :> adc_b1;
		c_adc :> adc_Temperature2;
		c_adc :> adc_Dummy;
		c_adc :> ExternalPot2;
	}


	return { current_a, current_b, adc_Temperature1, adc_Temperature2, adc_VoltageSupply, adc_Dummy, ExternalPot1, ExternalPot2 };

}

void do_adc_calibration_ad7949(chanend c_adc, calib_data &I_calib)
{
	unsigned int a, b;
	int i = 0;
	I_calib.Ia_calib = 0;
	I_calib.Ib_calib = 0;
	while ( i < ADC_CALIB_POINTS)
	{
		/* get ADC reading */
		c_adc <: ADC_CURRENT_REQ;

		slave
		{
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

//	printstr("ia_calibration ");   printint(I_calib.Ia_calib); printstr("\n");
//	printstr("ib_calibration ");   printint(I_calib.Ib_calib); printstr("\n");
}

{int, int} get_adc_calibrated_current_ad7949(chanend c_adc, calib_data &I_calib)
{
	// 0A equals 2.5V --> 9999
	unsigned int a, b;
	int Ia, Ib;

	c_adc <: ADC_CURRENT_REQ;

	slave
	{
		c_adc :> a;
		c_adc :> b;
	}

	Ia = (int) a - I_calib.Ia_calib;
	Ib = (int) b - I_calib.Ib_calib;

	return { Ia, Ib };
}


{int, int} get_adc_external_ad7949(chanend c_adc)
{

	int p1, p2;

	c_adc <: ADC_EXTERNAL_POT;

	slave
	{
		c_adc :> p1;
		c_adc :> p2;
	}

	return {p1, p2};
}

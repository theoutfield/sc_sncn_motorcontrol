//#include <print.h>
#include <adc_ad7949.h>

calib_data I_calib;
#define ADC_CURRENT_REQ	1
#define ADC_ALL_REQ	2
#define ADC_CALIB_POINTS 64
#define ADC_EXTERNAL_POT 3
#define Factor 6

int iFilterSUM_Ia;
int iFilterSUM_Ib;
int Ia=0;
int Ib=0;

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

void do_adc_calibration_ad7949( chanend c_adc )
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

{int, int} get_adc_calibrated_current_ad7949( chanend c_adc )
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


{int, int} get_adc_external_potentiometer_ad7949(chanend c_adc)
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

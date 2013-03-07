#include <print.h>
#include <adc_ad7949.h>

#define ADC_CALIB_POINTS 64
#define Factor 6

calib_data I_calib;

void do_adc_calibration_ad7949( chanend c_adc )
{
	unsigned a, b;
	unsigned xx1,xx2,xx3,xx4;

	int i = 0; I_calib.Ia_calib = 0; I_calib.Ib_calib = 0;
	while ( i < ADC_CALIB_POINTS)
	{
		/* get ADC reading */
		c_adc <: 0;

		slave {
			c_adc :> a;
			c_adc :> b;
			c_adc :> xx1;  // dummy readings
			c_adc :> xx2;
			c_adc :> xx3;
			c_adc :> xx4;
			c_adc :> xx1;
			c_adc :> xx2;
			c_adc :> xx3;
			c_adc :> xx4;
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

	printstr("ia_calibration ");   printint(I_calib.Ia_calib); printstr("\n");
	printstr("ib_calibration ");   printint(I_calib.Ib_calib); printstr("\n");

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


{int,int,int,int,int,int,int,int,int,int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc )
{
  //const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

  unsigned a, b;
  int Ia, Ib;
  unsigned adc_a1, adc_a2, adc_a3, adc_a4;
  unsigned adc_b1, adc_b2, adc_b3, adc_b4;

  c_adc <: 0;

  slave {
    c_adc :> a;
    c_adc :> b;
    c_adc :> adc_a1;
    c_adc :> adc_a2;
    c_adc :> adc_a3;
    c_adc :> adc_a4;
    c_adc :> adc_b1;
    c_adc :> adc_b2;
    c_adc :> adc_b3;
    c_adc :> adc_b4;
  }

  Ia = 	(int) a - I_calib.Ia_calib;		//-(Ib + Ic);		/* a + b + c = 0 */  //Ib = (int) b - I_calib.Ia_calib;  //Ic = (int) c - I_calib.Ib_calib;
  Ib = 	(int) b - I_calib.Ib_calib;

  return { Ia, Ib, adc_a1, adc_a2, adc_a3, adc_a4, adc_b1, adc_b2, adc_b3, adc_b4 };
}

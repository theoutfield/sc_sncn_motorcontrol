#include <print.h>
#include <adc_ad7949.h>

calib_data I_calib;
#define ADC_CALIB_POINTS 64
#define Factor 6

int iAdcNrReadings=0;
int iFilterSUM_Ia;
int iFilterSUM_Ib;
int Ia, Ib;

{int, int, int, int, int, int, int, int, int, int} get_adc_calibrated_ad7949(chanend c_adc, int iPwmOnOff)
{
  //const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

  int current_a, current_b;
  int adc_a1, adc_Temperature1, adc_VoltageSupply, ExternalPot1;
  int adc_b1, adc_Temperature2, adc_Dummy, ExternalPot2;


  c_adc <: 0;
   slave {
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

  if(iAdcNrReadings < 5){
	  iFilterSUM_Ia = current_a * 256;
	  iFilterSUM_Ib = current_b * 256;
  }
  if(iAdcNrReadings++ > 512) iAdcNrReadings=256;

  if(iPwmOnOff==0)
  {
	  iFilterSUM_Ia 	-= I_calib.Ia_calib;
	  iFilterSUM_Ia 	+= current_a;
	  I_calib.Ia_calib   = iFilterSUM_Ia/256;

	  iFilterSUM_Ib 	-= I_calib.Ib_calib;
	  iFilterSUM_Ib 	+= current_b;
	  I_calib.Ib_calib 	 = iFilterSUM_Ib/256;
  }


  Ia = 	current_a - I_calib.Ia_calib;
  Ib = 	current_b - I_calib.Ib_calib;

  //return { Ia, Ib, adc_a1, adc_a2, adc_a3, adc_a4, adc_b1, adc_b2, I_calib.Ia_calib, I_calib.Ib_calib };
  return { Ia, Ib, I_calib.Ia_calib, I_calib.Ib_calib, adc_Temperature1, adc_Temperature2, adc_VoltageSupply, adc_Dummy, ExternalPot1, ExternalPot2 };

}

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

  Ia = 	(int) a - I_calib.Ia_calib;
  Ib = 	(int) b - I_calib.Ib_calib;

  return { Ia, Ib };
}

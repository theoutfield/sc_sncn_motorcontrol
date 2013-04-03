#include <print.h>
#include <adc_ad7949.h>

calib_data I_calib;
int iAdcNrReadings=0;
int iFilterSUM_Ia;
int iFilterSUM_Ib;
int Ia, Ib;

{int,int,int,int,int,int,int,int,int,int} get_adc_calibrated_ad7949(chanend c_adc, int iPwmOnOff)
{
  //const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

  int current_a, current_b;

  int adc_a1, adc_a2, adc_a3, adc_a4;
  int adc_b1, adc_b2, adc_b3, adc_b4;

  c_adc <: 0;
   slave {
	    c_adc :> current_a;
	    c_adc :> current_b;
	    c_adc :> adc_a1;
	    c_adc :> adc_a2;
	    c_adc :> adc_a3;
	    c_adc :> adc_a4;
	    c_adc :> adc_b1;
	    c_adc :> adc_b2;
	    c_adc :> adc_b3;
	    c_adc :> adc_b4;
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

  //return { Ia, Ib, adc_a1, adc_a2, adc_a3, adc_a4, adc_b1, adc_b2, adc_b3, adc_b4 };
  return { Ia, Ib, adc_a1, adc_a2, adc_a3, adc_a4, adc_b1, adc_b2, I_calib.Ia_calib, I_calib.Ib_calib };


}
{int,int} get_adc_calibrated_cur_ad7949( chanend c_adc)
{
	  int current_a, current_b;

	  int adc_a1, adc_a2, adc_a3, adc_a4;
	  int adc_b1, adc_b2, adc_b3, adc_b4;

	  c_adc <: 0;
	   slave {
		    c_adc :> current_a;
		    c_adc :> current_b;
		    c_adc :> adc_a1;
		    c_adc :> adc_a2;
		    c_adc :> adc_a3;
		    c_adc :> adc_a4;
		    c_adc :> adc_b1;
		    c_adc :> adc_b2;
		    c_adc :> adc_b3;
		    c_adc :> adc_b4;
	   }

	  Ia = 	current_a - I_calib.Ia_calib;
	  Ib = 	current_b - I_calib.Ib_calib;

	  //return { Ia, Ib, adc_a1, adc_a2, adc_a3, adc_a4, adc_b1, adc_b2, adc_b3, adc_b4 };
	  return { Ia, Ib };
}

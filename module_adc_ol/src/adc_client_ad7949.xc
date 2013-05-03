#include <adc_ad7949.h>



{int,int,int,int,int,int,int,int,int,int} get_adc_calibrated_ad7949(chanend c_adc, int iPwmOnOff)
{
  //const int zero_amps = 9999; 	// 0A equals 2.5V --> 9999

  int Ia, Ib;

  int adc_a1, adc_Temperature1, adc_VoltageSupply, ExternalPoti1;
  int adc_b1, adc_Temperature2, adc_Dummy, ExternalPoti2;


  c_adc <: iPwmOnOff;
   slave {
	    c_adc :> Ia;
	    c_adc :> Ib;
	    c_adc :> adc_a1;
	    c_adc :> adc_Temperature1;
	    c_adc :> adc_VoltageSupply;
	    c_adc :> ExternalPoti1;
	    c_adc :> adc_b1;
	    c_adc :> adc_Temperature2;
	    c_adc :> adc_Dummy;
	    c_adc :> ExternalPoti2;
   }

  return { Ia, Ib, adc_a1, adc_b1, adc_Temperature1, adc_Temperature2, adc_VoltageSupply, adc_Dummy, ExternalPoti1, ExternalPoti2 };
}


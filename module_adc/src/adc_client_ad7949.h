#pragma once

{unsigned, unsigned} get_adc_vals_raw_ad7949( chanend c_adc );
{int,int,int,int,int,int,int,int,int,int} get_adc_calibrated_ad7949( chanend c_adc, int iPwmOnOff);

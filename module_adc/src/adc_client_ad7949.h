#pragma once

{unsigned, unsigned} get_adc_vals_raw_ad7949( chanend c_adc );
{int, int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc );
void do_adc_calibration_ad7949( chanend c_adc );

/**
 * \file adc_client_ad7949.h
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

#pragma once

{unsigned, unsigned} get_adc_vals_raw_ad7949( chanend c_adc );
{int,int,int,int,int,int,int,int,int,int} get_adc_calibrated_ad7949( chanend c_adc, int iPwmOnOff);

/*
 * old adc functions
 */
{int, int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc );
void do_adc_calibration_ad7949( chanend c_adc );

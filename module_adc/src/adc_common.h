/*
 * \file adc_common.h
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/
#ifndef __ADC_COMMON_H__
#define __ADC_COMMON_H__

typedef struct calibration {
	int Ia_calib;
	int Ib_calib;
	int Ic_calib;
} calib_data;

#endif /* __ADC_COMMON_H__ */

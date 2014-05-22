
/**
 * \file adc_common.h
 * \brief ADC Common Definitions
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#ifndef __ADC_COMMON_H__
#define __ADC_COMMON_H__

typedef struct calibration {
	int Ia_calib;
	int Ib_calib;
	int Ic_calib;
} calib_data;

#endif /* __ADC_COMMON_H__ */

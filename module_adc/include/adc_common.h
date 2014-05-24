
/**
 * \file adc_common.h
 * \brief ADC Common Definitions
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
*/

 

#ifndef __ADC_COMMON_H__
#define __ADC_COMMON_H__

typedef struct calibration {
	int Ia_calib;
	int Ib_calib;
	int Ic_calib;
} calib_data;

#endif /* __ADC_COMMON_H__ */

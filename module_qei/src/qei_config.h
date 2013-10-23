/**
 * File:    qei_config.h
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
#ifndef __QEI_CONFIG_H__
#define __QEI_CONFIG_H__

#define FILTER_LENGTH_QEI 8
#define FILTER_LENGTH_QEI_PWM 8
#define QEI_RPM_CONST 1000*60
#define QEI_PWM_RPM_CONST 18000*60
#define QEI_RAW_POS_REQ 1
#define QEI_ABSOLUTE_POS_REQ 2
#define QEI_VELOCITY_REQ 3
#define QEI_VELOCITY_PWM_RES_REQ 4
#define SYNC 5
#define SET_OFFSET 6

#endif /* __QEI_CONFIG_H__ */

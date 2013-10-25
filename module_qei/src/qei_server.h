/*
 * File:    qei_server.h
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
 */


#ifndef __QEI_SERVER_H__
#define __QEI_SERVER_H__
#include <xs1.h>
#include <dc_motor_config.h>
#include "qei_config.h"
#include "internal_config.h"
#include <refclk.h>
#include "filter_blocks.h"

/**
 * \brief Implementation of the QEI server thread (with index/no index)
 *
 * \channel c_qei_p1 the control channel for reading qei position in order of priority (highest) 1 ... (lowest) 4
 * \channel c_qei_p2 the control channel for reading qei position priority - 2
 * \channel c_qei_p3 the control channel for reading qei position priority - 3
 * \channel c_qei_p4 the control channel for reading qei position priority - 4
 *
 * \port p_qei the hardware port where the quadrature encoder is located
 *
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 *
 */
void run_qei(chanend c_qei_p1, chanend c_qei_p2, chanend c_qei_p3, chanend c_qei_p4, chanend c_qei_p5, port in p_qei, qei_par &qei_params);

#endif /*__QEI_SERVER_H__ */

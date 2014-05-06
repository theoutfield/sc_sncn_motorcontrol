
/**
 * \file qei_server.h
 * \brief QEI Sensor Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 */

#ifndef __QEI_SERVER_H__
#define __QEI_SERVER_H__
#include <xs1.h>
#include <bldc_motor_config.h>
#include "qei_config.h"
#include "internal_config.h"
#include <refclk.h>
#include "filter_blocks.h"

/**
 * \brief Implementation of the QEI server thread (for sensor with index/no index)
 *
 *	Output channel
 * \channel c_qei_p1 the control channel for reading qei position in order of priority (highest) 1 ... (lowest) 5
 * \channel c_qei_p2 the control channel for reading qei position priority - 2
 * \channel c_qei_p3 the control channel for reading qei position priority - 3
 * \channel c_qei_p4 the control channel for reading qei position priority - 4
 * \channel c_qei_p5 the control channel for reading qei position priority - 5
 * \channel c_qei_p6 the control channel for reading qei position priority - 6
 *
 *	Input port
 * \port p_qei the hardware port where the quadrature encoder is located
 *
 *	Input
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 *
 */
void run_qei(chanend c_qei_p1, chanend c_qei_p2, chanend c_qei_p3, chanend c_qei_p4, chanend c_qei_p5, \
		chanend c_qei_p6, port in p_qei, qei_par &qei_params);
//void run_qei(chanend c_qei[6], port in p_qei, qei_par &qei_params);

#endif /*__QEI_SERVER_H__ */

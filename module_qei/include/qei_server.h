/**
 * @file qei_server.h
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <qei_config.h>

 /**
* @brief Structure containing hall sensor port/s
*/
typedef struct {
    port p_qei;
} EncoderPorts;

/**
 * @brief initialize QEI sensor
 *
 * @param qei_params struct defines the resolution for quadrature encoder (QEI),
 *          gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par & qei_params);

/**
 * @brief Implementation of the QEI server thread (for sensor with index/no index)
 *
 * @param c_qei_p1 the control channel for reading qei position in order of priority (highest) 1 ... (lowest) 5
 * @param c_qei_p2 the control channel for reading qei position priority - 2
 * @param c_qei_p3 the control channel for reading qei position priority - 3
 * @param c_qei_p4 the control channel for reading qei position priority - 4
 * @param c_qei_p5 the control channel for reading qei position priority - 5
 * @param c_qei_p6 the control channel for reading qei position priority - 6
 * @param EncoderPorts structure containing the hardware port where the quadrature encoder is located
 * @param qei_params the structure defines sensor type and resolution parameters for qei
 */
void run_qei(chanend ? c_qei_p1, chanend ? c_qei_p2, chanend ? c_qei_p3,
             chanend ? c_qei_p4, chanend ? c_qei_p5, chanend ? c_qei_p6,
             EncoderPorts & encoder_ports, qei_par & qei_params);


/**
 * @file qei_client.h
 * @brief QEI Sensor Client Functions
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <qei_config.h>

/**
 * @brief Get position from QEI Server
 *
 * @param c_qei for communicating with the QEI Server
 * @param qei_params the struct defines sensor type and resolution parameters for qei
 *
 * @return  position from qei sensor in the range [0 - log(encoder_resolution)/log(2)]
 * @return  valid for qei with index sensors: not valid - 0/ valid - 1
 */
{unsigned int, unsigned int} get_qei_position(chanend c_qei,
        qei_par &qei_params);

/**
 * @brief Get absolute position from QEI Server
 *
 * @param[in] c_qei for communicating with the QEI Server
 *
 * @return  counted up position from qei sensor (incorporates set max ticks) in the range [ -max ticks to +max ticks]
 *
 * @return  direction of rotation, clockwise : 1 / anti-clockwise : -1
 */
{int, int} get_qei_position_absolute(chanend c_qei);

/**
 * @brief Calculates the velocity from QEI sensor in 1 ms loop
 *
 * @param c_qei for communicating with the QEI Server
 * @param qei_params the struct defines sensor type and resolution parameters for qei
 * @param qei_velocity_params struct for velocity calculation

 * @return velocity from qei sensor in rpm
 */
//int get_qei_velocity(chanend c_qei, qei_par &qei_params,
//        qei_velocity_par &qei_velocity_params);



/**
 * @brief Internal function to calculate QEI position information
 *
 * @param real_counts qei counts per rotation

 * @return  max position from qei sensor
 */
extern int __qei_max_counts(int real_counts);

/**
 * @brief Internal function
 */
{int, int, int} get_qei_sync_position(chanend c_qei);


/**
 * @brief Internal function
 */
void set_qei_sync_offset(chanend c_qei, int offset_forward, int offset_backward);

void reset_qei_count(chanend c_qei, int offset);


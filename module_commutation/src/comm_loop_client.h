/**
 * \file comm_loop_client.h
 * \brief Commutation Loop Client functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

/**
 * \brief Struct for commutation parameters
 */
typedef struct {
    int angle_variance;
    int max_speed_reached;
    int qei_forward_offset;
    int qei_backward_offset;
    int offset_forward;
    int hall_offset_clk;
    int hall_offset_cclk;
    int winding_type;
    int flag;
} commutation_par;

/**
 * \brief Sets input voltage for commutation loop
 *
 * \Output
 * \param c_commutation A chanend connected to commutation server
 *
 * \Input
 * \param input_voltage Motor input voltage (range: -13739 to 13739)
 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage);

/**
 * \brief Internal function used to set the commutation parameters
 *
 * \Output
 * \param c_commutation A chanend connected to commutation server
 *
 * \Input
 * \param commutation_params struct defines the commutation angle parameters
 */
void set_commutation_params(chanend c_commutation, commutation_par &commutation_params);

/**
 * \brief Selects the sensor type used for commutation (Hall sensor or quadrature encoder)
 *
 * \param c_commutation A chanend connected to commutation server
 *
 * \Input
 * \param sensor_select Sensor type, specify by using define HALL or QEI
 */
void set_commutation_sensor(chanend c_commutation, int sensor_select);

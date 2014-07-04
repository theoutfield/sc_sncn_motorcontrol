/**
 * @file commutation_client.h
 * @brief Commutation Client functions
 * @author Pavan Kanajar <pkanajar@synapticon.com>
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

/**
 * @brief Struct for commutation parameters
 */
typedef struct {
    int angle_variance;         /* max allowed variance depending on speed */
    int max_speed_reached;      /* FIXME: should be named rated_speed */
    int qei_forward_offset;
    int qei_backward_offset;
    int hall_offset_clk;
    int hall_offset_cclk;
    int winding_type;
} commutation_par;

/**
 * @brief Sets input voltage for commutation loop
 *
 * @param[out] c_commutation A chanend connected to commutation server
 *
 * @param[in] input_voltage Motor input voltage (range: -13739 to 13739)
 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage);

/**
 * @brief Internal function used to set the commutation parameters
 *
 * @param[out] c_commutation A chanend connected to commutation server
 *
 * @param[in] commutation_params struct defines the commutation angle parameters
 */
void set_commutation_params(chanend c_commutation, commutation_par &commutation_params);

/**
 * @brief Selects the sensor type used for commutation (Hall sensor or quadrature encoder)
 *
 * @param[out] c_commutation A chanend connected to commutation server
 *
 * @param[in] sensor_select Sensor type, specify by using define HALL or QEI
 */
void set_commutation_sensor(chanend c_commutation, int sensor_select);

void disable_motor(chanend c_commutation);

void enable_motor(chanend c_commutation);

int check_fet_state(chanend c_commutation);

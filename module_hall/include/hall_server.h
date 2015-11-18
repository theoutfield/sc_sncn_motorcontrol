/**
 * @file hall_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16
#define PULL_PERIOD_USEC 12

/**
 * Client/server interaction commands/tokens
 */
enum {
    HALL_POS_REQ,         //!< Position request token
    HALL_ABSOLUTE_POS_REQ,//!< Position request token
    HALL_VELOCITY_REQ,    //!< Velocity request token
    HALL_RESET_COUNT_REQ,     //!< Reset hall server ticks count
    HALL_FILTER_PARAM_REQ,//!< Filter length request token
    HALL_REQUEST_PORT_STATES
};


/**
 * @brief Structure definition for hall sensor
 */
typedef struct {
    int pole_pairs;
    int max_ticks_per_turn;
    int max_ticks;
    int sensor_polarity;
} hall_par;


/**
* @brief Structure containing hall sensor port/s
*/
#ifdef __XC__
typedef struct {
    port p_hall;
} HallPorts;



interface HallInterface {
    unsigned get_hall_pinstate();
    int get_hall_position();
    {int, int} get_hall_position_absolute();
    int get_hall_velocity();
    void reset_hall_count(int offset);
};


/**
 * @brief initialize hall sensor
 *
 * @param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par & hall_params);

/**
 * @brief A basic hall sensor server
 *
 * @param[out] c_hall_p1 the control channel for reading hall position in order of priority (highest) 1 ... (lowest) 6
 * @param[out] c_hall_p2 the control channel for reading hall position (priority 2)
 * @param[out] c_hall_p3 the control channel for reading hall position (priority 3)
 * @param[out] c_hall_p4 the control channel for reading hall position (priority 4)
 * @param[out] c_hall_p5 the control channel for reading hall position (priority 5)
 * @param[out] c_hall_p6 the control channel for reading hall position (priority 6)
 * @param[in] hall_ports structure containing the ports for reading the hall sensor data
 * @param hall_params structure defines the pole-pairs and gear ratio
 */

[[combinable]]
void run_hall(interface HallInterface server i_hall[5], HallPorts & hall_ports);
#endif

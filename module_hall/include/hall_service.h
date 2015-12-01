/**
 * @file hall_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ERROR                    0
#define SUCCESS                  1

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16
#define PULL_PERIOD_USEC 12

#define HALL_TICKS_PER_TURN 4096
/**
 * @brief Structure definition for hall sensor
 */
typedef struct {
    int pole_pairs;
    int max_ticks_per_turn;
    int max_ticks;
    int sensor_polarity;
} HallConfig;


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
    HallConfig getHallConfig();
};

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
 * @param hall_config structure defines the pole-pairs and gear ratio
 */

[[combinable]]
void hall_service(interface HallInterface server i_hall[5], HallPorts & hall_ports, HallConfig & hall_config);

#endif

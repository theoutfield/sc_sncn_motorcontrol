/**
 * @file hall_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define HALL                    1

#define ERROR                   0
#define SUCCESS                 1

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16
#define PULL_PERIOD_USEC 12

#define HALL_TICKS_PER_TURN 4096

/**
 * Structure definition for hall sensor
 */
typedef struct {
    int pole_pairs; /**< Lorem ipsum... */
    int max_ticks_per_turn; /**< Lorem ipsum... */
    int max_ticks; /**< Lorem ipsum... */
    int sensor_polarity; /**< Lorem ipsum... */
} HallConfig;


#ifdef __XC__
/**
* Structure containing hall sensor port/s
*/
typedef struct {
    port p_hall; /**< Lorem ipsum... */
} HallPorts;

/**
 * @brief Lorem ipsum...
 */
interface HallInterface {
    /**
     * @brief Gets hall pinstate
     *
     * @return Lorem ipsum...
     */
    unsigned get_hall_pinstate();
    /**
     * @brief Gets position from Hall Server
     *
     * @return the position in the range [0 - 4095] which maps to [0 - 359]/pole-pairs
     */
    int get_hall_position();

    /**
     * @brief Gets absolute position from Hall Server
     *
     * @return the counted up position (compensates for pole-pairs) in the range [0 - 4095] * pole-pairs
     * @return the counted up position (compensates for pole-pairs) in the range [0 - 4095] * pole-pairs
     */
    {int, int} get_hall_position_absolute();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_hall_velocity();
    /**
     * @brief Lorem ipsum...
     *
     * @param offset Lorem ipsum...
     */
    void reset_hall_count(int offset);
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    HallConfig getHallConfig();
    /**
     * @brief Lorem ipsum...
     *
     * @param in_config Lorem ipsum...
     */
    void setHallConfig(HallConfig in_config);
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int checkBusy();
};

/**
 * @brief Lorem ipsum...
 *
 * @param hall_ports Lorem ipsum...
 * @param hall_config Lorem ipsum...
 * @param i_hall[5] Lorem ipsum...
 */
[[combinable]]
void hall_service(HallPorts & hall_ports, HallConfig & hall_config,
                    interface HallInterface server i_hall[5]);

#endif

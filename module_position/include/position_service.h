/*
 * position_service.h
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#pragma once

/**
 * @brief Definition for referring to the BiSS sensor.
 */
#define BISS_SENSOR                4
#define CONTELEC_SENSOR            6

/**
 * @brief Structure type to define the BiSS Service configuration.
 */
typedef struct {
    int multiturn_length;       /**< Number of bits used for multiturn data */
    int multiturn_resolution;   /**< Number of bits of multiturn resolution */
    int singleturn_length;      /**< Number of bits used for singleturn data */
    int singleturn_resolution;  /**< Number of bits of singleturn resolution */
    int status_length;          /**< Rumber of bits used for status data */
    int crc_poly;               /**< CRC polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100 */
    int pole_pairs;             /**< Number of poles pairs to compute the electrical angle from the mechanical angle*/
    int polarity;               /**< Polarity, invert the direction */
    int clock_dividend;         /**< BiSS output clock frequency dividend */
    int clock_divisor;          /**< BiSS output clock frequency divisor */
    int timeout;                /**< Timeout after a BiSS read in clock ticks */
    int velocity_loop;          /**< Velocity loop time in microseconds */
    int max_ticks;              /**< The count is reset to 0 if greater than this */
    int offset_electrical;      /**< Offset for the electrical angle */
    int enable_push_service;
} BISSConfig;


/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    int multiturn_resolution;   /**< Multiturn resolution in bits. */
    int resolution_bits;        /**< Encoder resolution in bits. */
    int polarity;               /**< Encoder polarity. */

    int pole_pairs;             /**< Number of pole pairs (1-7) */

    int offset;                 /**< Rotary sensor offset (Zero) */

    int timeout;                /**< timeout after a read (in clock ticks) */

    int velocity_loop;          /**< Velcity loop time in microseconds */

    int max_ticks;              /**< The count is reset to 0 if greater than this */

    int filter;                 /**< filter parameter for contelect encoder */
    int enable_push_service;
} CONTELECConfig;


typedef struct {
    int sensor_type[2];
    BISSConfig biss_config;
    CONTELECConfig contelec_config;
} PositionConfig;


#ifdef __XC__

#include <spi_master.h>
#include <memory_manager.h>


/**
 * @brief Structure type to define the BiSS Service ports.
 */
typedef struct {
    port ?p_biss_data;   /**< Port for BiSS Interface signal input. */
    port ?p_biss_clk;    /**< Port for BiSS Interface clock output. */
    clock ?clk;          /**< Hardware clock used as time reference */
} BISSPorts;


typedef struct
{
    spi_master_interface spi_interface;
    out port ?slave_select;
} SPIPorts;

typedef struct
{
    BISSPorts biss_ports;
    SPIPorts spi_ports;
} PositionPorts;

interface PositionInterface
{
    /**
     * @brief Notifies the interested parties that a new notification
     * is available.
     */
    [[notification]]
    slave void notification();

    /**
     * @brief Provides the type of notification currently available.
     *
     * @return type of the notification
     */
    [[clears_notification]]
    int get_notification();

    unsigned int get_angle(void);

    { int, unsigned int } get_position(void);

    { int, unsigned int, unsigned int } get_real_position(void);

    int get_velocity(void);

    PositionConfig get_config(void);

    void set_config(PositionConfig in_config);

    void set_position(int in_count);

    unsigned int set_angle(unsigned int in_angle);

    unsigned int send_command(int opcode, int data, int data_bits);
};

void position_service(PositionPorts &position_ports, PositionConfig &position_config,
                      client interface shared_memory_interface ?i_shared_memory,
                      server interface PositionInterface i_position[3]);

#endif

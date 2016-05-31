/*
 * position_feedback_service.h
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#pragma once

#include <biss_config.h>
#include <contelec_config.h>

typedef struct {
    int sensor_type;
    BISSConfig biss_config;
    CONTELECConfig contelec_config;
} PositionFeedbackConfig;


#ifdef __XC__

#include <spi_master.h>

interface PositionFeedbackInterface
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

    PositionFeedbackConfig get_config(void);

    void set_config(PositionFeedbackConfig in_config);

    void set_position(int in_count);

    unsigned int set_angle(unsigned int in_angle);

    unsigned int send_command(int opcode, int data, int data_bits);
};


typedef struct
{
    port ?p_biss_data;   /**< Port for BiSS Interface signal input. */
    port ?p_biss_clk;    /**< Port for BiSS Interface clock output. */
    out port ?slave_select;
    spi_master_interface spi_interface;
} PositionFeedbackPorts;

#include <memory_manager.h>
#include <biss_service.h>
#include <contelec_service.h>


void position_feedback_service(PositionFeedbackPorts &?position_feedback_ports_1, PositionFeedbackConfig &?position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface ?i_position_feedback_1[3],
                               PositionFeedbackPorts &?position_feedback_ports_2, PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface ?i_position_feedback_2[3]);

#endif

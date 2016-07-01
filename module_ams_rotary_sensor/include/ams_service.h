/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once


#ifdef __XC__

#include <stdint.h>
#include <refclk.h>
#include <position_feedback_service.h>
#include <spi_master.h>
#include <memory_manager.h>

//interface AMSInterface
//{
//
//    /**
//     * @brief Notifies the interested parties that a new notification
//     * is available.
//     */
//    [[notification]]
//    slave void notification();
//
//    /**
//     * @brief Provides the type of notification currently available.
//     *
//     * @return type of the notification
//     */
//    [[clears_notification]]
//    int get_notification();
//
//    { unsigned int, int, int } get_ams_angle_velocity_position(void);
//
//    { int, unsigned int } get_ams_position(void);
//
//    unsigned int get_ams_real_position(void);
//
//    int get_ams_velocity(void);
//
//    AMSConfig get_ams_config(void);
//
//    void set_ams_config(AMSConfig in_config);
//
//    void reset_ams_position(int in_count);
//
//    unsigned int reset_ams_angle(unsigned int in_angle);
//
//    { int, int, unsigned int, unsigned int, unsigned int } get_ams_all();
//};

void initRotarySensorInterface(PositionFeedbackPorts &position_feedback_ports);
int initRotarySensor(PositionFeedbackPorts &position_feedback_ports, AMSConfig config);

//reading fx
//non-volatile regs
int readZeroPosition(PositionFeedbackPorts &position_feedback_ports);
int readNumberPolePairs(PositionFeedbackPorts &position_feedback_ports);
int readSettings1(PositionFeedbackPorts &position_feedback_ports);
int readSettings2(PositionFeedbackPorts &position_feedback_ports);
int readRedundancyReg(PositionFeedbackPorts &position_feedback_ports);

//volatile regs
int readProgrammingReg(PositionFeedbackPorts &position_feedback_ports);
int readCORDICMagnitude(PositionFeedbackPorts &position_feedback_ports);
int readRotaryDiagnosticAndAutoGainControl(PositionFeedbackPorts &position_feedback_ports);
int readRotarySensorError(PositionFeedbackPorts &position_feedback_ports);
int readRotarySensorAngleWithoutCompensation(PositionFeedbackPorts &position_feedback_ports);
int readRotarySensorAngleWithCompensation(PositionFeedbackPorts &position_feedback_ports);

//writing fx
int writeSettings1(PositionFeedbackPorts &position_feedback_ports, unsigned short data);
int writeSettings2(PositionFeedbackPorts &position_feedback_ports, unsigned short data);
int writeZeroPosition(PositionFeedbackPorts &position_feedback_ports, unsigned short data);
int writeNumberPolePairs(PositionFeedbackPorts &position_feedback_ports, unsigned short data);

[[combinable]]
void ams_service(PositionFeedbackPorts &position_feedback_ports, AMSConfig &config, client interface shared_memory_interface ?i_shared_memory,
        server interface PositionFeedbackInterface i_position_feedback[3]);

#endif

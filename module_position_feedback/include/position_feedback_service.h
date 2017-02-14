/*
 * position_feedback_service.h
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#pragma once

#include <biss_config.h>
#include <rem_16mt_config.h>
#include <rem_14_config.h>

#include <biss_struct.h>
#include <rem_16mt_struct.h>
#include <rem_14_struct.h>
#include <hall_struct.h>
#include <qei_struct.h>

#include <stdint.h>

#define NUMBER_OF_GPIO_PORTS   4    /**< Defines number of Digital IOs available. */

typedef enum {
    GPIO_INPUT=0,
    GPIO_INPUT_PULLDOWN=1,
    GPIO_OUTPUT=2
} GPIOType;

typedef struct {
    GPIOType port_0;
    GPIOType port_1;
    GPIOType port_2;
    GPIOType port_3;
} GPIOConfig;

typedef struct {
    int sensor_type;
    int polarity;   /**< Encoder polarity. >*/
    int pole_pairs; /**< Number of pole pairs (1-7) >*/
    int resolution; /**< number of ticks per turn >*/
    int offset;     /**< position offset in ticks, can be singleturn or multiturn depending on the sensor >*/
    int enable_push_service; /**< Select which data to push to shared memory >*/
    int ifm_usec;   /**< number of clock ticks in a microsecond >*/
    int velocity_compute_period; /**< velocity compute period in microsecond. Is also the polling period to write to the shared memory >*/
    int max_ticks; /**< reset multiturn to 0 when reached >*/
    BISSConfig biss_config;
    REM_16MTConfig rem_16mt_config;
    REM_14Config rem_14_config;
    QEIConfig qei_config;
    GPIOType gpio_config[4];
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

    { int, unsigned int, unsigned int } get_position(void);

    int get_velocity(void);

    PositionFeedbackConfig get_config(void);

    void set_config(PositionFeedbackConfig in_config);

    void set_position(int in_count);

    unsigned int send_command(int opcode, int data, int data_bits);

    int gpio_read(int gpio_num);

    void gpio_write(int gpio_num, int in_value);

    void exit();
};

typedef struct {
    spi_master_interface spi_interface;
    port * movable slave_select;
} SPIPorts;

typedef struct {
    in port p_qei_hall; /**< 4-bit Port for Encoder, BiSS or Hall signals input. */
} QEIHallPort;

typedef struct {
    out port ?p_hall_enc_select; /**< [Nullable] Port to control the signal input circuitry (if applicable in your SOMANET device). Also used for the BiSS clock output */
} HallEncSelectPort;

#include <memory_manager.h>
#include <biss_service.h>
#include <rem_16mt_service.h>
#include <rem_14_service.h>
#include <hall_service.h>
#include <qei_service.h>


void position_feedback_service(QEIHallPort &?qei_hall_port_1, QEIHallPort &?qei_hall_port_2, HallEncSelectPort &?hall_enc_select_port, SPIPorts &?spi_ports, port ?gpio_port_0, port ?gpio_port_1, port ?gpio_port_2, port ?gpio_port_3,
                               PositionFeedbackConfig &position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3]);

int tickstobits(uint32_t ticks);

void multiturn(int &count, int last_position, int position, int ticks_per_turn);

void switch_ifm_freq(PositionFeedbackConfig &position_feedback_config);

void write_shared_memory(client interface shared_memory_interface ?i_shared_memory, int enable_push_service, int count, int velocity, int angle, int hall_state);

int velocity_compute(int difference, int timediff, int resolution);

int gpio_read(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number);

void gpio_write(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number, int value);

void gpio_shared_memory(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory);


#endif

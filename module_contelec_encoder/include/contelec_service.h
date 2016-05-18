/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#include <stdint.h>
#include <refclk.h>

//encoder config
#define CONTELEC_OFFSET          0//3696
#define CONTELEC_POLARITY        CONTELEC_POLARITY_NORMAL//CONTELEC_POLARITY_NORMAL
#define CONTELEC_USEC            USEC_FAST
#define CONTELEC_TIMEOUT         40*CONTELEC_USEC
#define CONTELEC_RESOLUTION      16
#define CONTELEC_VELOCITY_LOOP   1000
#define CONTELEC_FILTER          0x05

//SPI config
#define DEFAULT_SPI_CLOCK_DIV 32        // 250/DIV MHz
#define SPI_MASTER_MODE 1 //clock active high
#define SPI_MASTER_SD_CARD_COMPAT 1 //MOSI high during input

#define CONTELEC_SENSOR 6

#define ERROR       0
#define SUCCESS     1

#define CONTELEC_POLARITY_NORMAL      0
#define CONTELEC_POLARITY_INVERTED    1


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
} CONTELECConfig;


#ifdef __XC__

#include <spi_master.h>

typedef struct
{
    spi_master_interface spi_interface;
    out port slave_select;
} SPIPorts;

interface CONTELECInterface
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

    { unsigned int, int, int, unsigned int } get_contelec_angle_velocity(void);

    { int, unsigned int } get_contelec_position(void);

    { int, unsigned int, unsigned int } get_contelec_real_position(void);

    int get_contelec_velocity(void);

    CONTELECConfig get_contelec_config(void);

    void set_contelec_config(CONTELECConfig in_config);

    void reset_contelec_position(int in_count);

    unsigned int reset_contelec_angle(unsigned int in_angle);

    unsigned int command_contelec(int opcode, int data, int data_bits);
};

int contelec_encoder_init(SPIPorts &contelec_ports, CONTELECConfig config);
void init_spi_ports(SPIPorts &spi_ports);


[[combinable]]
void contelec_service(SPIPorts &contelec_ports, CONTELECConfig &config, interface CONTELECInterface server i_contelec[5]);

{ char, int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &contelec_ports);

void contelec_encoder_write(SPIPorts &contelec_ports, int opcode, int data, int data_bits);

#endif

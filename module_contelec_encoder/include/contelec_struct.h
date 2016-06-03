/*
 * contelec_struct.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#include <refclk.h>

#define CONTELEC_SENSOR                6

#define CONTELEC_USEC            USEC_FAST

#define SPI_MASTER_MODE 1 //clock active high
#define SPI_MASTER_SD_CARD_COMPAT 1 //MOSI high during input

#define ERROR       0
#define SUCCESS     1

#define CONTELEC_POLARITY_NORMAL      0
#define CONTELEC_POLARITY_INVERTED    1

#define CONTELEC_CTRL_RESET         0x00
#define CONTELEC_CONF_DIR           0x55
#define CONTELEC_CONF_NULL          0x56
#define CONTELEC_CONF_PRESET        0x57
#define CONTELEC_CONF_STPRESET      0x50
#define CONTELEC_CONF_MTPRESET      0x59
#define CONTELEC_CONF_FILTER        0x5B
#define CONTELEC_CALIB_TBL_SIZE     0x3D
#define CONTELEC_CALIB_TBL_POINT    0x3E
#define CONTELEC_CTRL_SAVE          0x1C


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

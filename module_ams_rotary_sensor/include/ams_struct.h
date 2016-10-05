/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#include <user_config.h>

#define AMS_USEC            IFM_TILE_USEC

#define AMS_SENSOR      5

#define ERROR       0
#define SUCCESS     1

#define AS5147      0
#define AS5047      1

#define AMS_SENSOR_TYPE AS5047

#define SPI_MASTER_MODE 1

#define AMS_POLARITY_NORMAL      0
#define AMS_POLARITY_INVERTED    1

#define AMS_PWM_OFF     0
#define AMS_PWM_ON      1

#define AMS_ABI_RES_11BIT   0
#define AMS_ABI_RES_10BIT   1

#define AMS_ABI_ON_PWM_W    0
#define AMS_UVW_ON_PWM_I    1

#define AMS_DAE_ON          0
#define AMS_DAE_OFF         1

#define AMS_DATA_DAECANG    0
#define AMS_DATA_CORDICANG       1

#define AMS_NOISE_NORMAL    0
#define AMS_NOISE_REDUCED   1

typedef enum {
    AMS_HYS_11BIT_3LSB = 0,
    AMS_HYS_11BIT_2LSB = 1,
    AMS_HYS_11BIT_1LSB = 2,
    AMS_HYS_11BIT_OFF = 3,

    AMS_HYS_10BIT_3LSB = 3,
    AMS_HYS_10BIT_2LSB = 0,
    AMS_HYS_10BIT_1LSB = 1,
    AMS_HYS_10BIT_OFF = 2
} AMS_Hysteresis;


/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    int resolution_bits;        /**< Encoder resolution in bits. */
#if AMS_SENSOR_TYPE == AS5147
    int width_index_pulse;      /**< Width of the index pulse I (0 = 3LSB, 1 = 1LSB). */
#else
    int factory_settings;       /**< Factory Settings, just reading, no  writing. */
#endif
    int noise_setting;          /**< Noise setting. In 3.3V operation, VDD and VREG must be tied together. In this
                                     configuration, normal noise performance (ONL) is available at
                                     reduced maximum temperature (125°C) by clearing NOISESET
                                     to 0. When NOISESET is set to 1, the full temperature range is
                                     available with reduced noise performance (ONH). */
    int polarity;               /**< Encoder polarity. */
    int uvw_abi;                /**< Defines the PWM Output (0 = ABI is operating, W is used as PWM;
                                     1 = UVW is operating, I is used as PWM) */
    int dyn_angle_comp;         /**< Disable Dynamic Angle Error Compensation
                                     (0 = DAE compensation ON, 1 = DAE compensation OFF) */
    int data_select;            /**< This bit defines which data can be read form address
                                     16383dec (3FFFhex). 0->DAECANG 1->CORDICANG */
    int pwm_on;                 /**< Enables PWM (setting of UVW_ABI Bit necessary) */
    int pole_pairs;             /**< Number of pole pairs (1-7) */
    AMS_Hysteresis hysteresis;  /**< Hysteresis for 11 Bit ABI Resolution:
                                     0 = 3 LSB
                                     1 = 2 LSB
                                     2 = 1 LSB
                                     3 = no hysteresis

                                     Hysteresis for 10 Bit ABI Resolution:
                                     0 = 2 LSB
                                     1 = 1 LSB
                                     2 = no hysteresis LSB
                                     3 = 3 LSB) */
    int abi_resolution;         /**< Resolution of ABI (0 = 11 bits, 1 = 10 bits) */

    int offset;                 /**< Rotary sensor offset (Zero) */

    int cache_time;             /**< How long to cache the position (in clock ticks) */

    int velocity_loop;          /**< Velcity loop time in microseconds */

    int max_ticks;              /**< The count is reset to 0 if greater than this */

    int enable_push_service;
} AMSConfig;


#define AMS_MAX_RESOLUTION   16384

//volatile registers
#define ADDR_ERRFL      0x0001
#define ADDR_PROG       0x0003
#define ADDR_DIAAGC     0x3FFC
#define ADDR_MAG        0x3FFD
#define ADDR_ANGLEUNC   0x3FFE
#define ADDR_ANGLECOM   0x3FFF

//non-volatile registers
#define ADDR_ZPOSM      0x0016
#define ADDR_ZPOSL      0x0017
#define ADDR_SETTINGS1  0x0018
#define ADDR_SETTINGS2  0x0019
#define ADDR_RED        0x001A

//COMMAND MASKS
#define WRITE_MASK 0xBFFF
#define READ_MASK 0x4000

//DATA MASKS
#define BITS_14_MASK    0x3FFF
#define BITS_12_MASK    0x0FFF
#define BITS_8_MASK     0x00FF
#define BITS_7_MASK     0x007F
#define BITS_6_MASK     0x003F
#define BITS_5_MASK     0x001F
#define BITS_3_MASK     0x0007

#define POLE_PAIRS_ZERO_MASK 0xFFF8
#define POLE_PAIRS_SET_MASK  0x0007
#define ROTATION_SENSE_CW_MASK  0x
#define ROTATION_SENSE_CCW_MASK 0x

//RETURN VALUES
#define SUCCESS_WRITING  1
#define PARITY_ERROR    -1
#define ERROR_WRITING   -2

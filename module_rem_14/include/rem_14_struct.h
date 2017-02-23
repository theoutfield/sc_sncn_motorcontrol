/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#define REM_14_SENSOR_RESOLUTION      (1<<14)

#define AS5147      0
#define AS5047      1

#define REM_14_PWM_OFF     0
#define REM_14_PWM_ON      1

#define REM_14_ABI_ON_PWM_W    0
#define REM_14_UVW_ON_PWM_I    1

#define REM_14_DATA_DAECANG    0
#define REM_14_DATA_CORDICANG  1

typedef enum {
    REM_14_ABI_RES_11BIT=0,
    REM_14_ABI_RES_10BIT=1
} REM_14_ABIResolution;

typedef enum {
    REM_14_DAE_ON=0,
    REM_14_DAE_OFF=1
} REM_14_DynAngleComp;

typedef enum {
    REM_14_NOISE_NORMAL=0,
    REM_14_NOISE_REDUCED=1
} REM_14_Noise;

typedef enum {
    REM_14_HYS_11BIT_3LSB = 0,
    REM_14_HYS_11BIT_2LSB = 1,
    REM_14_HYS_11BIT_1LSB = 2,
    REM_14_HYS_11BIT_OFF = 3,

    REM_14_HYS_10BIT_3LSB = 3,
    REM_14_HYS_10BIT_2LSB = 0,
    REM_14_HYS_10BIT_1LSB = 1,
    REM_14_HYS_10BIT_OFF = 2
} REM_14_Hysteresis;


/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    REM_14_Noise noise_setting;          /**< Noise setting. In 3.3V operation, VDD and VREG must be tied together. In this
                                             configuration, normal noise performance (ONL) is available at
                                             reduced maximum temperature (125°C) by clearing NOISESET
                                             to 0. When NOISESET is set to 1, the full temperature range is
                                             available with reduced noise performance (ONH). */
    REM_14_DynAngleComp dyn_angle_comp;  /**< Disable Dynamic Angle Error Compensation
                                              (0 = DAE compensation ON, 1 = DAE compensation OFF) */
    REM_14_Hysteresis hysteresis;        /**< Hysteresis for 11 Bit ABI Resolution:
                                             0 = 3 LSB
                                             1 = 2 LSB
                                             2 = 1 LSB
                                             3 = no hysteresis

                                             Hysteresis for 10 Bit ABI Resolution:
                                             0 = 2 LSB
                                             1 = 1 LSB
                                             2 = no hysteresis LSB
                                             3 = 3 LSB) */
    REM_14_ABIResolution abi_resolution; /**< Resolution of ABI (0 = 11 bits, 1 = 10 bits) */
} REM_14Config;


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
#define REM_14_SUCCESS_WRITING  1
#define REM_14_PARITY_ERROR    -1
#define REM_14_ERROR_WRITING   -2

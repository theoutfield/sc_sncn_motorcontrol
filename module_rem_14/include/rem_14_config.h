/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <rem_14_struct.h>

#define DEFAULT_SPI_CLOCK_DIV     32                   /**<  divisor for SPI clock frequency, (250/DIV)/2 MHz */
#define REM_14_POLLING_TIME       30                   /**< Time between reads in micro seconds */

#define REM_14_SENSOR_TYPE        AS5047
#define SPI_MASTER_MODE           1
#define REM_14_EXECUTING_TIME     2                    /**< fraction of microsecond: 1 us / 2 = 0.5 us */
#define REM_14_SAVING_TIME        5                    /**< fraction of microsecond: 1 us / 5 = 0.2 us */
#define REM_14_WIDTH_INDEX_PULSE  0                    /**< Width of the index pulse I (0 = 3LSB, 1 = 1LSB). */
#define REM_14_FACTORY_SETTINGS   1                    /**< Factory Settings, just reading, no  writing. */
#define REM_14_UVW_ABI            REM_14_ABI_ON_PWM_W  /**< Defines the PWM Output (0 = ABI is operating, W is used as PWM;
                                                            1 = UVW is operating, I is used as PWM) */


#define REM_14_DATA_SELECT        REM_14_DATA_DAECANG  /**< This bit defines which data can be read form address
                                                            16383dec (3FFFhex). 0->DAECANG 1->CORDICANG */


#define REM_14_PWM_CONFIG         REM_14_PWM_OFF       /**< Enables PWM (setting of UVW_ABI Bit necessary) */

/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <rem_14_struct.h>

#define REM_14_OFFSET          0
#define REM_14_POLARITY        REM_14_POLARITY_NORMAL//REM_14_POLARITY_NORMAL
#define REM_14_CACHE_TIME      (60*REM_14_USEC)
#define REM_14_RESOLUTION      14
#define REM_14_VELOCITY_LOOP   30
#define DEFAULT_SPI_CLOCK_DIV 32        // 250/DIV MHz
#define REM_14_SENSOR_EXECUTING_TIME (REM_14_USEC/2)       //0.5 us
#define REM_14_SENSOR_SAVING_TIME    (REM_14_USEC/5)       //0.2 us

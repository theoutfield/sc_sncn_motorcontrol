/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <ams_struct.h>

#define AMS_OFFSET          0
#define AMS_POLARITY        AMS_POLARITY_NORMAL//AMS_POLARITY_NORMAL
#define AMS_CACHE_TIME      (60*AMS_USEC)
#define AMS_RESOLUTION      14
#define AMS_VELOCITY_LOOP   30
#define DEFAULT_SPI_CLOCK_DIV 32        // 250/DIV MHz
#define AMS_SENSOR_EXECUTING_TIME (AMS_USEC/2)       //0.5 us
#define AMS_SENSOR_SAVING_TIME    (AMS_USEC/5)       //0.2 us

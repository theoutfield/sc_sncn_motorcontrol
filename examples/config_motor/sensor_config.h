// default sensor config

#pragma once

//BiSS config, use default if not set before
#ifndef BISS_CONFIG
#define BISS_MULTITURN_RESOLUTION  10
#define BISS_SINGLETURN_RESOLUTION 18
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION + 1 //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     18
#define BISS_MAX_TICKS             0x7fffffff   // the count is reset to 0 if greater than this
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequency: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         32           // supported frequencies are (tile frequency) / 2n
#define BISS_VELOCITY_LOOP         50         // velocity loop time in microseconds
#define BISS_TIMEOUT               20*IFM_TILE_USEC // BiSS timeout in clock ticks
#define BISS_CLOCK_PORT            BISS_CLOCK_PORT_EXT_D5
#define BISS_DATA_PORT             BISS_DATA_PORT_2
#endif

//REM 16MT config
#define REM_16MT_FILTER            0x02
#define REM_16MT_USE_TIMESTAMP
#ifdef REM_16MT_USE_TIMESTAMP
#define REM_16MT_TIMEOUT           10*IFM_TILE_USEC
#define REM_16MT_VELOCITY_LOOP     53
#else
#define REM_16MT_TIMEOUT           38*IFM_TILE_USEC
#define REM_16MT_VELOCITY_LOOP     65
#endif

//REM 14 config
#define REM_14_CACHE_TIME          (60*IFM_TILE_USEC)
#define REM_14_VELOCITY_LOOP       30

//QEI config
#define QEI_SENSOR_INDEX_TYPE      QEI_WITH_INDEX
#define QEI_SENSOR_SIGNAL_TYPE     QEI_RS422_SIGNAL
#define QEI_SENSOR_RESOLUTION      1000


// Sensor resolution (count per revolution)

//Manual setting
//#define COMMUTATION_SENSOR_RESOLUTION   65536
//#define FEEDBACK_SENSOR_RESOLUTION      65536

//auto set commutation and feedback sensor resolution depending on sensor used
#ifndef COMMUTATION_SENSOR_RESOLUTION
#if (MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
#define COMMUTATION_SENSOR_RESOLUTION (1<<BISS_SINGLETURN_RESOLUTION)
#elif (MOTOR_COMMUTATION_SENSOR == REM_16MT_SENSOR)
#define COMMUTATION_SENSOR_RESOLUTION (1<<16)
#elif (MOTOR_COMMUTATION_SENSOR == REM_14_SENSOR)
#define COMMUTATION_SENSOR_RESOLUTION (1<<14)
#elif (MOTOR_COMMUTATION_SENSOR == HALL_SENSOR)
#define COMMUTATION_SENSOR_RESOLUTION 4096*POLE_PAIRS
#elif (MOTOR_COMMUTATION_SENSOR == QEI_SENSOR)
#define COMMUTATION_SENSOR_RESOLUTION QEI_SENSOR_RESOLUTION
#endif
#endif

#ifndef FEEDBACK_SENSOR_RESOLUTION
#if (MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
#define FEEDBACK_SENSOR_RESOLUTION (1<<BISS_SINGLETURN_RESOLUTION)
#elif (MOTOR_FEEDBACK_SENSOR == REM_16MT_SENSOR)
#define FEEDBACK_SENSOR_RESOLUTION (1<<16)
#elif (MOTOR_FEEDBACK_SENSOR == REM_14_SENSOR)
#define FEEDBACK_SENSOR_RESOLUTION (1<<14)
#elif (MOTOR_FEEDBACK_SENSOR == HALL_SENSOR)
#define FEEDBACK_SENSOR_RESOLUTION 4096*POLE_PAIRS
#elif (MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
#define FEEDBACK_SENSOR_RESOLUTION QEI_SENSOR_RESOLUTION
#endif
#endif

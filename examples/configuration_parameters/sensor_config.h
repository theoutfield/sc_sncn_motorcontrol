// default sensor config

#pragma once

//General config
#define SENSOR_MAX_TICKS                  0x7fffffff   // the count is reset to 0 if greater than this
#define SENSOR_VELOCITY_COMPUTE_PERIOD    1000         // default velocity compute period 1ms

//BiSS config, use default if not set before
#ifndef BISS_CONFIG
#define BISS_MULTITURN_RESOLUTION  10
#define BISS_SINGLETURN_RESOLUTION 18
#define BISS_SENSOR_RESOLUTION     (1<<BISS_SINGLETURN_RESOLUTION)
#define BISS_FILLING_BITS          0
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^6 is 0b110000
#define BISS_CLOCK_FREQUENCY       4000         // BiSS output clock frequency in kHz
#define BISS_SENSOR_VELOCITY_COMPUTE_PERIOD 100 // velocity loop time in microseconds (will be rounded to the next multiple of the polling time)
#define BISS_TIMEOUT               20           // BiSS timeout in microseconds
#define BISS_BUSY                  30           // maximum number of bits to read before the start bit (= maximum duration of ACK bit in clock cycles)
#define BISS_CLOCK_PORT            BISS_CLOCK_PORT_EXT_D5
#define BISS_DATA_PORT_NUMBER      ENCODER_PORT_2 // [ENCODER_PORT_1, ENCODER_PORT_2]
#endif

//REM 16MT config
#define REM_16MT_FILTER            0x02
#define REM_16MT_SENSOR_VELOCITY_COMPUTE_PERIOD     400 // velocity loop time in microseconds (will be rounded to the next multiple of the polling time)

//REM 14 config
#define REM_14_SENSOR_HYSTERESIS            REM_14_HYS_11BIT_2LSB;
#define REM_14_SENSOR_NOISE_SETTINGS        REM_14_NOISE_NORMAL;
#define REM_14_DYN_ANGLE_ERROR_COMPENSATION REM_14_DAE_ON;
#define REM_14_ABI_RESOLUTION_SETTINGS      REM_14_ABI_RES_11BIT;
#define REM_14_SENSOR_VELOCITY_COMPUTE_PERIOD       450 // velocity loop time in microseconds (will be rounded to the next multiple of the polling time)

//QEI config
#define QEI_SENSOR_NUMBER_OF_CHANNELS   QEI_WITH_INDEX     // [QEI_WITH_INDEX, QEI_WITH_NO_INDEX]
#define QEI_SENSOR_SIGNAL_TYPE          QEI_RS422_SIGNAL   // [QEI_RS422_SIGNAL, QEI_TTL_SIGNAL]
#define QEI_SENSOR_PORT_NUMBER          ENCODER_PORT_2     // [ENCODER_PORT_1, ENCODER_PORT_2]
#define QEI_SENSOR_VELOCITY_COMPUTE_PERIOD        1000
#define QEI_SENSOR_RESOLUTION           4000               // ticks per turn = 4 * CPR (Cycles per revolution)

//Hall config
#define HALL_SENSOR_PORT_NUMBER      ENCODER_PORT_1     // [ENCODER_PORT_1, ENCODER_PORT_2]
#define HALL_SENSOR_VELOCITY_COMPUTE_PERIOD       1000
#define HALL_SENSOR_RESOLUTION                    4096*MOTOR_POLE_PAIRS

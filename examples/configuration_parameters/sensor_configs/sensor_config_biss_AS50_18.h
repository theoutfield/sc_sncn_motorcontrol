// default sensor config

#pragma once

#define BISS_CONFIG

#define BISS_MULTITURN_RESOLUTION  0
#define BISS_SINGLETURN_RESOLUTION 18
#define BISS_SENSOR_RESOLUTION     (1<<BISS_SINGLETURN_RESOLUTION)
#define BISS_FILLING_BITS          0
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_CLOCK_FREQUENCY       4000         // BiSS output clock frequency in kHz
#define BISS_SENSOR_VELOCITY_COMPUTE_PERIOD 100 // velocity loop time in microseconds
#define BISS_TIMEOUT               20           // BiSS timeout in microseconds
#define BISS_BUSY                  30
#define BISS_CLOCK_PORT            BISS_CLOCK_PORT_EXT_D5
#define BISS_DATA_PORT_NUMBER      ENCODER_PORT_2

#define BISS_MULTITURN_RESOLUTION  12
#define BISS_SINGLETURN_RESOLUTION 13
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION + 1 //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION
#define BISS_FRAME_BYTES           (( (3 + 2 + BISS_MULTITURN_LENGTH + BISS_SINGLETURN_LENGTH + BISS_STATUS_LENGTH + 6) -1)/32 + 1) //at least 3 bits + ack and start bits + data + crc
#define BISS_POLARITY              BISS_POLARITY_NORMAL
#define BISS_MAX_TICKS             0x7fffffff   // the count is reset to 0 if greater than this
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_DATA_PORT_BIT         1            // bit number (0 = rightmost bit) when inputing from a multibit port
#define BISS_CLK_PORT_HIGH         (0b0100 | SET_PORT1_AS_QEI_PORT2_AS_HALL) // high clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLK_PORT_LOW          SET_PORT1_AS_QEI_PORT2_AS_HALL // low clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequercy: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         128          // supported frequencies are (tile frequency) / 2^n
#define BISS_USEC                  USEC_FAST    // number of ticks in a microsecond
#define BISS_VELOCITY_LOOP         1000         // velocity loop time in microseconds
#define BISS_TIMEOUT               14*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     1800

/**
 * @file biss_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>

/**
 * @brief Definition for referring to the BiSS sensor.
 */
#define BISS_SENSOR                3

#define ERROR                      0
#define SUCCESS                    1

#define SET_ALL_AS_QEI             0b0011
#define SET_PORT1_AS_HALL_PORT2_AS_QEI 0b0010
#define SET_PORT1_AS_QEI_PORT2_AS_HALL 0b0001

#define BISS_POLARITY_NORMAL       0
#define BISS_POLARITY_INVERTED     1

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
#define BISS_CLK_PORT_HIGH         1            // high clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLK_PORT_LOW          0            // low  clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequercy: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         128          // supported frequencies are (tile frequency) / 2^n
#define BISS_USEC                  USEC_FAST    // number of ticks in a microsecond
#define BISS_VELOCITY_LOOP         1000         // velocity loop time in microseconds
#define BISS_TIMEOUT               14*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     1800

/**
 * @brief Structure type to define the BiSS Service configuration.
 */
typedef struct {
    int multiturn_length;
    int multiturn_resolution;
    int singleturn_length;
    int singleturn_resolution;
    int status_length;
    int crc_poly;
    int poles;
    int polarity;
    int clock_dividend;
    int clock_divisor;
    int timeout;
    int velocity_loop;
    int max_ticks;
    int offset_electrical;
} BISSConfig;


/**
 * enum for the several status informations
 * of the read_biss_sensor_data() function
 */
enum {
  NoError,
  CRCError,
  NoAck,
  NoStartBit,
};

#ifdef __XC__

#include <platform.h>

/**
 * @brief Structure type to define the BiSS Service ports.
 */
typedef struct {
    port ?p_biss_config; /**< [Nullable] Port to control the signal input circuitry (if applicable in your SOMANET device). */
    port p_biss_data;   /**< Port for BiSS Interface signal input. */
    port p_biss_clk;    /**< Port for BiSS Interface clock output. */
    clock clk;          /**< Clock for BiSS Interface. */
} BISSPorts;

/**
 * @brief Interface type to communicate with the BiSS Service.
 */
interface BISSInterface {

    /**
     * @brief Get position from BiSS Server
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_biss_position();

    /**
     * @brief Get real internal encoder position from BiSS Server
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_biss_real_position();

    /**
     * @brief Get only singleturn position without CRC checking from BiSS Server
     *
     * @return singleturn position
     */
    unsigned int get_biss_position_fast();

    /**
     * @brief Get electrical angle from BiSS Server
     *
     * @return electrical angle
     */
    unsigned int get_biss_angle_electrical();

    /**
     * @brief Get velocity from BiSS Server
     *
     * @return velocity
     */
    int get_biss_velocity();

    /**
     * @brief Get biss parameters from BiSS Server
     *
     * @return biss parameters
     */
    BISSConfig get_biss_config();

    /**
     * @brief Set count of the BiSS Server
     *
     * @param count to set
     *
     */
    void reset_biss_position(int count);

    /**
     * @brief Set electrical angle of the BiSS Server
     *
     * @param electrical angle to set
     *
     * @return electrical angle offset
     */
    unsigned int reset_biss_angle_electrical(unsigned int angle);

    /**
     * @brief Set biss parameters of the BiSS Server
     *
     * @param biss parameters to set
     *
     */
    void set_biss_config(BISSConfig in_config);

    /**
     * @brief Set calib flag in the BiSS Server which will alway return 0 as electrical angle
     *
     */
    void set_biss_calib(int flag);
};


/**
 * @brief Service to read and process data from an Feedback BiSS Encoder Sensor.
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputing the clock.
 * @param biss_config structure definition for the biss encoder data lengths, crc polynom, clock frequency, etc
 * @param[out] i_biss array of interfaces to send the data to the client
 */
void biss_service(BISSPorts & biss_ports, BISSConfig & biss_config, interface BISSInterface server i_biss[5]);


/**
 * @brief Read generic BiSS sensor data
 *
 * @param p_biss_clk 1-bit out port to output the biss clock
 * @param p_biss_data in port for reading the biss encoder data
 * @param clk clock to generate the biss clock
 * @param a the dividend of the desired clock rate
 * @param b the divisor of the desired clock rates
 * @param[out] data array to store the received bits
 * @param data_length length of sensor data in bits (without crc)
 * @param frame_bytes number of 32 bit bytes to read from the encoder, should be able to contain 2 bits + ack and start bits + data + crc
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return error status (NoError, CRCError, NoAck, NoStartBit)
 */
unsigned int read_biss_sensor_data(port out p_biss_clk, port in p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly);


/**
 * @brief Read up to 32 bit of BiSS sensor data without CRC checking
 *
 * @param p_biss_clk 1-bit out port to output the biss clock
 * @param p_biss_data in port for reading the biss encoder data
 * @param clk clock to generate the biss clock
 * @param a the dividend of the desired clock rate
 * @param b the divisor of the desired clock rates
 * @param before_length length of data to dismiss
 * @param data_length length of desired data
 *
 * @return data
 */
unsigned int read_biss_sensor_data_fast(port out p_biss_clk, port in p_biss_data, clock clk, unsigned a, unsigned b, int before_length, int data_length);


/**
 * @brief Extract turn data from a biss encoder sensor data
 *
 * @param data biss sensor data
 * @param biss_config structure definition for the biss encoder data lengths
 *
 * @return absolute count
 * @return position in the range [0 - (2^singleturn_resolution - 1)]
 * @return status bits (error and warning bits), 0 = ok
 */
{ int, unsigned int, unsigned int } biss_encoder(unsigned int data[], BISSConfig biss_config);


/**
 * @brief Compute a crc for biss data
 *
 * @param data biss data
 * @param data_length length of data in bits
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return inverted crc for biss
 */
unsigned int biss_crc(unsigned int data[], unsigned int data_length, unsigned int poly);


/**
 * @brief Try 1-bit error correction for biss data
 *
 * @param[out] data biss data
 * @param data_length length of data in bits
 * @param frame_bytes number of 32 bit bytes of data
 * @param crc_received crc received with the data
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 */
void biss_crc_correct(unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int poly);

#endif

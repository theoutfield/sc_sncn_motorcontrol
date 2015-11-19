/**
 * @file qei_server.h
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

//#include <qei_config.h>
#define FILTER_LENGTH_QEI        8
#define FILTER_LENGTH_QEI_PWM    8

#define QEI_RPM_CONST            1000*60
#define QEI_PWM_RPM_CONST        18000*60

#define QEI_RAW_POS_REQ          1
#define QEI_ABSOLUTE_POS_REQ     2
#define QEI_VELOCITY_REQ         3
#define QEI_VELOCITY_PWM_RES_REQ 4
#define SYNC                     5
#define SET_OFFSET               6
#define QEI_RESET_COUNT          7

/**
 * @brief struct definition for quadrature sensor
 */
typedef struct {
    int max_ticks_per_turn;
    int real_counts;
    int max_ticks;      // paramater allows for more turns
    int index;          // no_index - 0 index - 1
    int poles;
    int sensor_polarity;
} qei_par;

/**
 * @brief struct definition for velocity calculation from qei sensor
 */
typedef struct QEI_VELOCITY_PARAM
{
    int previous_position;
    int old_difference;
    int filter_buffer[8];
    int index;
    int filter_length;
} qei_velocity_par;


/**
 * @brief Internal function to calculate QEI position information
 *
 * @param real_counts qei counts per rotation

 * @return  max position from qei sensor
 */
extern int __qei_max_counts(int real_counts);

#ifdef __XC__

 /**
* @brief Structure containing hall sensor port/s
*/
typedef struct {
    port ?p_qei_config;
    port p_qei;
} EncoderPorts;

interface QEIInterface{

    {unsigned int, unsigned int} get_qei_position();
    {int, int} get_qei_position_absolute();
    {int, int, int} get_qei_sync_position();
    int get_qei_velocity();
    void set_qei_sync_offset(int, int);
    void reset_qei_count(int offset);

};


/**
 * @brief initialize QEI sensor
 *
 * @param qei_params struct defines the resolution for quadrature encoder (QEI),
 *          gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par & qei_params);

/**
 * @brief Initialize struct for velocity calculation from QEI sensor
 *
 * @param qei_velocity_params  struct is initialised
 */
void init_qei_velocity_params(qei_velocity_par &qei_velocity_params);

int calculate_qei_velocity(int count, qei_par &qei_config, qei_velocity_par &qei_velocity_params);
/**
 * @brief Implementation of the QEI server thread (for sensor with index/no index)
 *
 * @param c_qei_p1 the control channel for reading qei position in order of priority (highest) 1 ... (lowest) 5
 * @param c_qei_p2 the control channel for reading qei position priority - 2
 * @param c_qei_p3 the control channel for reading qei position priority - 3
 * @param c_qei_p4 the control channel for reading qei position priority - 4
 * @param c_qei_p5 the control channel for reading qei position priority - 5
 * @param c_qei_p6 the control channel for reading qei position priority - 6
 * @param EncoderPorts structure containing the hardware port where the quadrature encoder is located
 * @param qei_params the structure defines sensor type and resolution parameters for qei
 */
void run_qei(interface QEIInterface server i_qei[5],
             EncoderPorts & encoder_ports, qei_par qei_config, qei_velocity_par qei_velocity_params);

#endif

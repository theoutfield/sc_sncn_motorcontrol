/**
 * @file qei_server.h
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ERROR                    0
#define SUCCESS                  1

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

enum QEI_Polarity{ QEI_POLARITY_NORMAL = 0, QEI_POLARITY_INVERTED = 1}; /* Encoder polarity */
enum QEI_Type{ QEI_WITH_NO_INDEX = 0, QEI_WITH_INDEX = 1}; /* Encoder type */

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
} QEIConfig;


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
} QEIPorts;

interface QEIInterface{
    int checkBusy();
    {unsigned int, unsigned int} get_qei_position();
    {int, int} get_qei_position_absolute();
    {int, int, int} get_qei_sync_position();
    int get_qei_velocity();
    void set_qei_sync_offset(int, int);
    void reset_qei_count(int offset);
    QEIConfig getQEIConfig();
    void setQEIConfig(QEIConfig in_config);
};


/**
 * @brief initialize QEI sensor
 *
 * @param qei_params struct defines the resolution for quadrature encoder (QEI),
 *          gear-ratio, poles, encoder type
 */
/*
void init_qei_param(qei_par & qei_params);
*/
/**
 * @brief Initialize struct for velocity calculation from QEI sensor
 *
 * @param qei_velocity_params  struct is initialised
 */
/*
void init_qei_velocity_params(qei_velocity_par &qei_velocity_params);
*/
//int calculate_qei_velocity(int count, qei_config &qei_config, qei_velocity_par &qei_velocity_params);
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
void qei_service(interface QEIInterface server i_qei[5],
             QEIPorts & encoder_ports, QEIConfig qei_config);

#endif

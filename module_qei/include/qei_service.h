/**
 * @file qei_server.h
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define QEI_CHANGES_PER_TICK     4 //Quadrature encoder

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

#define QEI                      2
//#define QEI_WITH_NO_INDEX        3
//#define QEI_WITH_INDEX           4

#define QEI_PORT_AS_TTL           0b0000
#define QEI_PORT_AS_RS422         0b0010

typedef enum { QEI_POLARITY_NORMAL = 0, QEI_POLARITY_INVERTED = 1 } QEI_Polarity; /* Encoder polarity */
typedef enum { QEI_RS422_SIGNAL = 11, QEI_TTL_SIGNAL = 22 } QEI_SignalType;
typedef enum { QEI_WITH_NO_INDEX = 3, QEI_WITH_INDEX  = 4 } QEI_IndexType;
//enum QEI_Type{ QEI_WITH_NO_INDEX = 0, QEI_WITH_INDEX = 1}; /* Encoder type */

/**
 * @brief struct definition for quadrature sensor
 */
typedef struct {
    int ticks_resolution; //real_counts;
    QEI_IndexType index_type;          // no_index - 0 index - 1
    QEI_Polarity sensor_polarity;
    QEI_SignalType signal_type;
} QEIConfig;


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
void qei_service(QEIPorts & encoder_ports, QEIConfig qei_config,
                interface QEIInterface server i_qei[5]);

#endif

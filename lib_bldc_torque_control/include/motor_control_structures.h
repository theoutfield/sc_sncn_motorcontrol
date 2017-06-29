/*
 * motor_control_structures.h
 *
 *  Created on: Aug 2, 2016
 *      Author: ramin
 */


#ifndef MOTOR_CONTROL_STRUCTURES_H_
#define MOTOR_CONTROL_STRUCTURES_H_

/**
 * @brief Status of motor control related services
 */
typedef enum {
    INACTIVE= 1,/**< the task is started and can be used as a server. */
    ACTIVE = 2  /**< the task is not started yet. */
} TaskStatus;

/**
 * @brief Type for position sensors.
 */
typedef enum {
    HALL_SENSOR=1,
    QEI_SENSOR=2,
    BISS_SENSOR=4,
    REM_14_SENSOR=5,
    REM_16MT_SENSOR=6,
    SSI_SENSOR=7
} SensorType;

/**
 * @brief Type for Sensor Error
 *
 */
typedef enum {
    SENSOR_NO_ERROR=0,
    SENSOR_REM_16MT_WEAK_MAG_FIELD_ERROR       = 0x1,
    SENSOR_REM_16MT_MT_COUNTER_ERROR           = 0x2,
    SENSOR_REM_16MT_ST_CORDIC_ERROR            = 0x3,
    SENSOR_REM_16MT_MT_SPEED_OVERLOW_ERROR     = 0x4,
    SENSOR_REM_16MT_FILTER_CONFIG_ERROR        = 0xA,
    SENSOR_REM_16MT_FILTER_SPEED_OVERLOW_ERROR = 0xB,
    SENSOR_REM_16MT_UNKNOWN_CMD_ERROR          = 0xD,
    SENSOR_REM_16MT_CONFIG_ERROR               = 0xE,
    SENSOR_BISS_ERROR_BIT_ERROR                = 15,
    SENSOR_BISS_WARNING_BIT_ERROR              = 16,
    SENSOR_BISS_ERROR_AND_WARNING_BIT_ERROR    = 17,
    SENSOR_BISS_NO_ACK_BIT_ERROR               = 18,
    SENSOR_BISS_NO_START_BIT_ERROR             = 19,
    SENSOR_CHECKSUM_ERROR                      = 20,
    SENSOR_BISS_DATA_LINE_ERROR                = 21,
    QEI_INDEX_LOSING_TICKS                     = 22
} SensorError;

/**
 * @brief Type for Motion control Error
 *
 */
typedef enum {
    MOTION_CONTROL_NO_ERROR=0,
    MOTION_CONTROL_BRAKE_NOT_RELEASED=1
} MotionControlError;

/**
 * @brief Type for Watchdog Error
 *
 */
typedef enum {
    WATCHDOG_NO_ERROR                           = 0,
    WATCHDOG_TICKS_ERROR                        = 0b0001,
    WATCHDOG_DEAD_TIME_PHASE_A_ERROR            = 0b0010,
    WATCHDOG_DEAD_TIME_PHASE_B_ERROR            = 0b0011,
    WATCHDOG_DEAD_TIME_PHASE_C_ERROR            = 0b0100,
    WATCHDOG_DEAD_TIME_PHASE_D_ERROR            = 0b0101,
    WATCHDOG_OVER_CURRENT_ERROR                 = 0b0110,
    WATCHDOG_OVER_UNDER_VOLTAGE_OVER_TEMP_ERROR = 0b0111,
    WATCHDOG_UNKNOWN_ERROR                      = 0b1111
} WatchdogError;

/**
 * @brief Type for motors.
 */
typedef enum {
    BDC_MOTOR  = 10, /**< Brushed DC Motor. */
    BLDC_MOTOR = 11  /**< Brush-less DC Motor. */
} MotorType;

/**
 * @brief Commutation method.
 */
typedef enum {
    SINE = 20,  /**< Sine commutation. */
    FOC  = 21  /**< Vector control. */
} CommutationMethod;

/**
 * @brief Type for the kind of winding of a BLDC motor.
 */
typedef enum {
    STAR_WINDING=1, /**< Star winding. */
    DELTA_WINDING   /**< Delta winding. */
} BLDCWindingType;

/**
 * @brief Fixes matching of reference torque sign to position and velocity signs.
 */
typedef enum {
    MOTOR_PHASES_NORMAL   = 0,  /**< Normal connection  */
    MOTOR_PHASES_INVERTED = 1   /**< Flipped connection */
} MotorPhasesConfiguration;

/**
 * @brief Type of profiler
 */
typedef enum {
    LINEAR  = 1 /**< Linear profiler */
} MotionProfileType;


/**
 * @brief Fault Codes
 */
typedef enum {
    //having no fault:
    NO_FAULT=0,

    //standard defined faults (IEC-61800)
    DEVICE_INTERNAL_CONTINOUS_OVER_CURRENT_NO_1 = 0X2221,
    PHASE_FAILURE_L1                            = 0X3131,
    PHASE_FAILURE_L2                            = 0X3132,
    PHASE_FAILURE_L3                            = 0X3133,
    OVER_VOLTAGE_NO_1                           = 0X3211,
    UNDER_VOLTAGE_NO_1                          = 0X3221,
    EXCESS_TEMPERATURE_DRIVE                    = 0X4310,
    INCREMENTAL_SENSOR_1_FAULT                  = 0X7305,
    SPEED_FAULT                                 = 0X7310,
    POSITION_FAULT                              = 0X7320,

    //user specific faults
    WRONG_REF_CLK_FRQ                           = 0XFF01,
    HALL_SENSOR_FAULT                           = 0XFF02
} FaultCode;

/**
 * Structure type for Motorcontrol Service configuration.
 */
typedef struct {
    MotorType motor_type;                   /**< Type of motor to drive. */
    CommutationMethod commutation_method;   /**< Commutation method. */
    BLDCWindingType bldc_winding_type;      /**< Type of winding of your motor (if using a BLDC motor). */
    MotorPhasesConfiguration phases_inverted;   /**< Type of polarity of your motor. */
    int licence;                            /**< Licence number for using the library of module_advanced_foc  */
    SensorType commutation_sensor;          /**< Absolute position sensor used for commutation (if using a BLDC motor). For the moment just Hall sensor can be used [HALL_SENSOR]. */
    int ifm_tile_usec;
    int hall_offset[2];                     /**< Feedback Hall sensor error offset for positive (hall_offset[0]) and negative (hall_offset[1]) turning [0:4095]. (Often required to optimize commutation if using a BLDC motor). */
    int hall_state[6];                       /**< Hall port state while being in sector [1-6] */

    //variables added to be used in motor_control_service
    int pole_pairs;                        /**< motor pole pair*/
    int max_torque;                        /**< maximum motor torque*/
    int max_current;                       /**< maximum stator current*/
    int rated_current;                     /**< rated motor phase current*/
    int rated_torque;                      /**< rated motor torque*/
    int commutation_angle_offset;          /**< position offset (which is finally added to the value which is recived from position sensor to compensate the required angle shift)*/
    int torque_constant;                   /**< motor torque constant*/
    int torque_P_gain;                     /**< proportional constant in torque controller*/
    int torque_I_gain;                     /**< integral constant in torque controller*/
    int torque_D_gain;                     /**< derivative constant in torque controller*/
    int current_ratio;                     /**<ratio between current recieved in control core, and real phase current*/
    int voltage_ratio;                     /**<ratio between adc measured value and real dc-bus voltage*/
    int temperature_ratio;                 /**<ratio between adc measured value and IFM board temperature*/

    int percent_offset_torque;              // (maximum) generated torque while finding offset value as a percentage of rated torque

    int phase_resistance;                   /**< uOhm*/
    int phase_inductance;                   /**< uH*/
    int dc_bus_voltage;                     /**< dc bus voltage*/

    // regenerative mode variables
    int recuperation_enabled;

    int battery_e_max;  // maximum energy status of battery
    int battery_e_min;  // minimum energy status of battery
    int recuperation_p_max;    // maximum regenerative power (in Watts)
    int recuperation_p_min;    // minimum regenerative power (in Watts)
    int recuperation_speed_max;
    int recuperation_speed_min;

    // protection limits
    // comment: there are some definitions in standard dictionary (such as MAX_TORQUE, MAX_CURRENT, ...) but
    //          these values are for normal operation. (with high probability) the protection limits are different
    //          than these maximum values...
    int protection_limit_over_current;  //maximum tolerable value of phase current (under abnormal conditions)
    int protection_limit_over_voltage;  //maximum tolerable value of dc-bus voltage (under abnormal conditions)
    int protection_limit_under_voltage; //minimum tolerable value of dc-bus voltave (under abnormal conditions)
    int protection_limit_over_temperature; //maximum tolerable value of board temperature

    //cogging torque compensation
    int torque_offset [1024]; //values of torque to add to the torque command relative to the sensor position (in milliNm)

} MotorcontrolConfig;

/**
 * @brief Structure type to send the data from lower controlling levels
 * to higher controlling levels
 */
typedef struct
{
    int error_status;

    int computed_torque;
    int torque_set;

    int V_dc;
    int I_b;
    int I_c;

    unsigned int angle;
    unsigned int hall_state;
    unsigned int qei_index_found;
    int angle_velocity;
    SensorError angle_sensor_error;
    SensorError angle_last_sensor_error;

    int position;
    int singleturn;
    int velocity;
    SensorError  sensor_error;
    SensorError  last_sensor_error;
    unsigned int sensor_timestamp;

    int secondary_position;
    int secondary_singleturn;
    int secondary_velocity;
    SensorError  secondary_sensor_error;
    SensorError  secondary_last_sensor_error;
    unsigned int secondary_sensor_timestamp;

    MotionControlError motion_control_error;

    WatchdogError watchdog_error;

    int temperature;

    int analogue_input_a_1;
    int analogue_input_a_2;
    int analogue_input_b_1;
    int analogue_input_b_2;

    unsigned int gpio[4];
}UpstreamControlData;

/**
 * @brief Structure type to send the data from higher controlling levels
 * to higher controlling levels
 */
typedef struct
{
    int position_cmd;
    int velocity_cmd;
    int torque_cmd;
    int offset_torque;
}DownstreamControlData;

#endif /* MOTOR_CONTROL_STRUCTURES_H_ */

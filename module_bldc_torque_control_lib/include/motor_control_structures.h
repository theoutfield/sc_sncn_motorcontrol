/*
 * motor_control_structures.h
 *
 *  Created on: Aug 2, 2016
 *      Author: ramin
 */


#ifndef MOTOR_CONTROL_STRUCTURES_H_
#define MOTOR_CONTROL_STRUCTURES_H_

/**
 * @brief Status of motor control related task
 * ACTIVE   -> the task is started and can be used as a server
 * INACTIVE -> the task is not started yet
 */
typedef enum {
    INACTIVE= 1,
    ACTIVE = 2
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
    MOTOR_PHASES_NORMAL  = 1,  /**< Normal connection  */
    MOTOR_PHASES_INVERTED =-1   /**< Flipped connection */
} MotorPhasesConfiguration;

/**
 * @brief Type for the polarity of the position sensor
 */
typedef enum {
    LINEAR  = 1 /**< Linear profiler */
} MotionProfileType;


/**
 * @brief Type for the polarity of the position sensor
 */
typedef enum {
    NORMAL_POLARITY  = 1, /**< Normal polarity. */
    INVERTED_POLARITY=-1   /**< Inverted polarity. */
} PolarityType;

/**
 * @brief Fault Codes
 */
typedef enum {
    //having no fault:
    NO_FAULT=0,

    //standard defined faults (IEC-61800)
    DEVICE_INTERNAL_CONTINOUS_OVER_CURRENT_NO_1 = 0X2221,
    OVER_VOLTAGE_NO_1                           = 0X3211,
    UNDER_VOLTAGE_NO_1                          = 0X3221,

    //user specific faults
    WRONG_REF_CLK_FRQ=0XFF01
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
    int hall_state_angle[7];                 /**< estimated angle while being in sector [1-6] (the array is 7 for with other arrays in control_variables.h)*/

    //variables added to be used in motor_control_service
    int pole_pairs;                          /**< motor pole pair*/
    int max_torque;                         /**< maximum motor torque*/
    int max_current;                        /**< maximum stator current*/
    int rated_current;                      /**< rated motor phase current*/
    int rated_torque;                       /**< rated motor torque*/
    int commutation_angle_offset;           /**< position offset (which is finally added to the value which is recived from position sensor to compensate the required angle shift)*/
    int torque_constant;                    /**< motor torque constant*/
    int torque_P_gain;                     /**< proportional constant in torque controller*/
    int torque_I_gain;                     /**< integral constant in torque controller*/
    int torque_D_gain;                     /**< derivative constant in torque controller*/
    int current_ratio;                      // ratio between current recieved in control core, and real phase current
    int voltage_ratio;                      // ratio between adc measured value and real dc-bus voltage
    int percent_offset_torque;              // (maximum) generated torque while finding offset value as a percentage of rated torque

    int phase_resistance;                   /**< uOhm*/
    int phase_inductance;                   /**< uH*/
    int v_dc;                               /**< dc bus voltage*/

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

    unsigned int angle;
    unsigned int hall_state;
    int angle_velocity;

    int position;
    int velocity;

    int position_additional;
    int velocity_additional;

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

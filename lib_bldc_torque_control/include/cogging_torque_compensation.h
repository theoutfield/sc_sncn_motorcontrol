#pragma once


/**
 * @brief Standard resolution of the position received by the sensor
 */
#define STANDARD_SENSOR_RESOLUTION 65536

/**
 * @brief Size of the lookup table saving the cogging torque data
 */
#define COGGING_TORQUE_ARRAY_SIZE 1024


/**
 * @brief Structure type containing recording parameters of the cogging torque
 */
typedef struct
{
    int remaining_cells;
    int torque_recording[2*COGGING_TORQUE_ARRAY_SIZE];
    short counter_average[2*COGGING_TORQUE_ARRAY_SIZE];
    float torque_mean[2];
    int torque_recording_started;
    int back_and_forth;
    int count_start;
    short index_start[2];
    short start_threshold;
    short first_bin_flag;
    int number_turns;
    int position_step;
    int rotation_sign;
    int delay_counter;
    int velocity_reference;
}CoggingTorqueParam;

/**
 * @brief Initializes the structure of type CoggingTorqueParam to start the cogging torque recording procedure.
 *
 * @param parameters    structure of type CoggingTorqueParam which contains cogging torque parameters
 * @param velocity_ref          The reference velocity which will be used in torque recording procedure.
 *                              Note: velocity_ref should be low (not more than 10 RPM depending on the type of motor used
 *
 *  */
void init_cogging_torque_parameters(CoggingTorqueParam &parameters, int velocity_ref);

/**
 * @brief returns the cogging torque offset command to apply corresponding to the rotor position
 *
 * @param sensor_position   16-bit absolute mechanical position
 * @param array             reference to the lookup table containing the torque offsets
 *                          Note : array is the reference to a table with COGGING_TORQUE_ARRAY_SIZE members
 *
 * @return short            the function returns the value of the torque (in milliNm)
 *  */
short get_torque_offset (int sensor_position, int * array);

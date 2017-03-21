/**
 * @file profile_control.h
 * @brief Profile Control functions
 *  Implements position profile control, velocity profile control
 *  and torque profile control functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <motion_control_service.h>

/**
 * @brief Structure definition for Profiler configuration.
 */
typedef struct{
    int polarity;

    //Position
    int velocity;   /**< Default velocity for Position Profile ramps generation [RPM]. */
    int max_position;    /**< Max. reachable position. */
    int min_position;    /**< Min. reachable position. */
    int ticks_per_turn;  /**< Number of ticks per turn. */

    //Velocity
    int acceleration;    /**< Default acceleration for Velocity Profile ramps generation [RPM/s].  */
    int deceleration;    /**< Default deceleration for Velocity Profile ramps generation [RPM/s].  */
    int max_acceleration;    /**< Max. reachable acceleration [RPM/s].  */
    int max_deceleration;    /**< Max. reachable deceleration [RPM/s].  */
    int max_velocity;    /**< Max. reachable velocity [RPM].  */

    //Torque
    int current_slope;   /**< Default current variation for torque ramps [ADC Current ticks/s]. Check ADC Module to know more about ADC Current ticks. */
    int max_current_slope;  /**< Max. reachable current variation [ADC Current ticks/s]. Check ADC Module to know more about ADC Current ticks. */
    int max_current;     /**< Max reachable current [ADC ticks]. Check ADC Module to know more about ADC Current ticks. */
}ProfilerConfig;

/**
 * @brief Position Profiler Initializer. It sets the profiler configuration.
 *        It is required running this function once before start using the Profiler.
 *
 * @param profile_position_config Configuration for the Position Profiler.
 * @param i_position_control Communication interface to the Position Control Service.
 * @param i_hall Interface to hall Service
 * @param i_qei Interface to Incremental Encoder Service (QEI)
 * @param i_biss Interface to BiSS Encoder Service (QEI)
 */
//FIXME find a proper way to send the ticks per turn
void init_position_profiler(ProfilerConfig profile_position_config);

/**
 * @brief Velocity Profiler Initializer. It sets the profiler configuration.
 *        It is required running this function once before start using the Profiler.
 *
 * @param profile_velocity_config Configuration for the Velocity Profiler.
 * @param i_velocity_control Communication interface to the Velocity Control Service.
 */
//FIXME: to be implemented for the new position/velocity controller
//void init_velocity_profiler(ProfilerConfig profile_velocity_config,
//                                interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Torque Profiler Initializer. It sets the profiler configuration.
 *        It is required running this function once before start using the Profiler.
 *
 * @param profile_torque_config Configuration for the Torque Profiler.
 * @param i_torque_control Communication interface to the Torque Control Service.
 */
//void init_torque_profiler(ProfilerConfig profile_torque_config,
//                                interface TorqueControlInterface client i_torque_control);

/**
 * @brief Generates a profile ramp from the current position to the defined target position
 *        according to the provided parameters. Then sequentially sends each point of the profile
 *        as target to the Position Control Service.
 *
 * @param target_position New target position in [INT_MIN:INT_MAX].
 * @param velocity in [RPM].
 * @param acceleration in [RPM/s].
 * @param deceleration [RPM/s].
 * @param i_position_control Communication interface to the Position Control Service.
 */
void set_profile_position( DownstreamControlData &downstream_control_data, int velocity, int acceleration, int deceleration,
                           interface PositionVelocityCtrlInterface client i_position_control );

/**
 * @brief Set profile velocity with Velocity Control loop
 *
 * @param target_velocity is the new target velocity in (rpm)
 * @param acceleration in (rpm/s)
 * @param deceleration in (rpm/s)
 * @param i_velocity_control for communicating with the Velocity Control Server
 */
//FIXME: to be implemented for the new position/velocity controller
#if 0
void set_profile_velocity( int target_velocity, int acceleration, int deceleration,
                           interface VelocityControlInterface client i_velocity_control );
#endif

/**
 * @brief Set profile torque with Torque Control loop
 *
 * @param target_torque is the new target torque in (mNm * current resolution)
 * @param torque_slope in (mNm/s * current resolution)
 * @param i_torque_control for communicating with the Torque Control Server
 */
//FIXME
#if 0
void set_profile_torque( int target_torque, int torque_slope,
                         interface TorqueControlInterface client i_torque_control );
#endif

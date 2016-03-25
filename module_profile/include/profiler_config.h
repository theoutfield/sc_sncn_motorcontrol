/*
 * profiler_config.h
 *
 *  Created on: Mar 24, 2016
 *      Author: Synapticon
 */

#pragma once

/**
 * @brief Structure definition for Profiler configuration.
 */
typedef struct{
    int polarity;

    //Position
    int velocity;   /**< Default velocity for Position Profile ramps generation [RPM]. */
    int max_position;    /**< Max. reachable position. */
    int min_position;    /**< Min. reachable position. */

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
} ProfilerConfig;

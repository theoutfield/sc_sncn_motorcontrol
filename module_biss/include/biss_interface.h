/**
 * @file biss_interface.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <biss_config.h>


/**
 * @brief Interface definition for biss server
 */
interface i_biss {

    /**
     * @brief Get position from BiSS Server
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_position();

    /**
     * @brief Get real internal encoder position from BiSS Server
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_real_position();

    /**
     * @brief Get only singleturn position without CRC checking from BiSS Server
     *
     * @return singleturn position
     */
    unsigned int get_position_fast();

    /**
     * @brief Get electrical angle from BiSS Server
     *
     * @return electrical angle
     */
    unsigned int get_angle_electrical();

    /**
     * @brief Get velocity from BiSS Server
     *
     * @return velocity
     */
    int get_velocity();

    /**
     * @brief Get biss parameters from BiSS Server
     *
     * @return biss parameters
     */
    biss_par get_params();

    /**
     * @brief Set count of the BiSS Server
     *
     * @param count to set
     *
     */
    void set_count(int count);

    /**
     * @brief Set electrical angle of the BiSS Server
     *
     * @param electrical angle to set
     *
     * @return electrical angle offset
     */
    unsigned int set_angle_electrical(unsigned int angle);

    /**
     * @brief Set biss parameters of the BiSS Server
     *
     * @param biss parameters to set
     *
     */
    void set_params(biss_par biss_params);
};

/**
 * @file biss_interface.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <biss_config.h>
#include <bldc_motor_config.h>

/**
 * @brief Interface definition for biss server
 */
interface i_biss {
    { int, unsigned int, unsigned int } get_position();
    { int, unsigned int, unsigned int } get_real_position();
    unsigned int get_position_fast();
    unsigned int get_angle_electrical();
    int get_velocity();
    void set_params(biss_par biss_params);
    biss_par get_params();
    void set_count(int count);
};

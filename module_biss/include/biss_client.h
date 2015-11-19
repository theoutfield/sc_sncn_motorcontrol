/**
 * @file biss_client.h
 * @author Synapticon GmbH <support@synapticon.com>
 */
#pragma once

#include <biss_interface.h>

void init_biss_velocity_params(biss_velocity_par &biss_velocity_params);

int get_biss_velocity(client interface i_biss i_biss, biss_par & biss_params, biss_velocity_par &biss_velocity_params, int count);

/**
 * @brief Get position from BiSS Server
 *
 * @param[out] i_biss interface for communicating with the BiSS Server
 *
 * @return absolute position
 * @return singleturn position
 * @return error and warning bits from the encoder
 */
{ int, unsigned int, unsigned int } get_biss_state(client interface i_biss i_biss);

unsigned int get_biss_position(client interface i_biss i_biss);

int get_biss_position_absolute(client interface i_biss i_biss);

unsigned int get_biss_angle_electrical(client interface i_biss i_biss);

void set_biss_params(client interface i_biss i_biss, biss_par biss_params);

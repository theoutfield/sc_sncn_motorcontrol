/**
 * @file biss_client.h
 * @author Synapticon GmbH <support@synapticon.com>
 */
#pragma once

#include <biss_config.h>


/**
 * @brief Get position from BiSS Server
 *
 * @param[out] i_biss interface for communicating with the BiSS Server
 *
 * @return absolute position
 * @return singleturn position
 * @return error and warning bits from the encoder
 */
{ int, unsigned int, unsigned int } get_biss_position(client interface i_biss i_biss);

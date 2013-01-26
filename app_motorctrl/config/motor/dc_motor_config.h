/**
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-02-22 1850)
 * \
 */

#ifndef _DC_MOTOR_CONFIG__H_
#define _DC_MOTOR_CONFIG__H_

#pragma once

// Define the motor specs
// needed by module_ctrl_loops
#define POLE_PAIRS	7

#define GEAR_RATIO	156

#define MAX_NOMINAL_SPEED  4000   // in 1/min

#define MAX_NOMINAL_CURRENT  5    // in A

#endif

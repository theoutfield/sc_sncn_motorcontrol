/**
 * @file foc_config.h
 * @brief FOC configuration file
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define def_BOOST_RANGE 32
#define defBOOST 128

#define defADD 65536
#define RAMP_UMOT 65536 // 1 - 65536 max

/**
 * Structure type for Field Controller configuration
 */
typedef struct {
    int kP; /**< Proportional Gain. */
    int kI; /**< Integral Gain. */
    int field_e1;
    int field_e2;
    int field_out_p_part;
    int field_out_i_part;
} FieldControlParams;


/**
 * @file mc_internal_constants.h
 * @brief Internal Definitions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once
#include <pwm_common.h>

/* TODO: output of control loops shouldn't (directly) depend on
 * PWM_MAX_VALUE, etc. */
#define BLDC_PWM_CONTROL_LIMIT              (PWM_MAX_VALUE - PWM_DEAD_TIME) / 2 //(((PWM_MAX_VALUE) - (PWM_DEAD_TIME)) / 2)
#define BDC_PWM_CONTROL_LIMIT               (PWM_MAX_VALUE - PWM_DEAD_TIME)     //((PWM_MAX_VALUE) - (PWM_DEAD_TIME))

#define PWM_MIN_LIMIT 250 /* FIXME: remove it when proper PWM module is used */

#define HALL_POSITION_INTERPOLATED_RANGE    4096

/* FIXME: those should be moved to board support packages */
#define DC100_RESOLUTION                    740
#define DC300_RESOLUTION                    400
#define OLD_DC300_RESOLUTION                264

#define BLDC                                1
#define BDC                                 2

#define INIT_BUSY                           0
#define INIT                                1
#define CHECK_BUSY                          10

#define SENSOR_SELECT                       150

#define FILTER_SIZE                         8   // default (but used anywhere)
#define FILTER_SIZE_MAX                     128 // max size

#define SET_COMM_PARAM_ECAT                 20
#define SET_HALL_CONFIG_ECAT                21
#define SET_QEI_PARAM_ECAT                  22

#define INIT_VELOCITY_CTRL                  29
#define SET_VELOCITY_FILTER                 30

#define HALL_PRECISION                      2
#define QEI_PRECISION                       512

#define SET_CTRL_PARAMETER                  100

#define CONFIG_DIO_INPUT                    10
#define CONFIG_DIO_DONE                     15
#define GPIO_INPUT                          20
#define GPIO_OUTPUT                         22

/* Digital Input types */
#define GP_INPUT_TYPE                       40
#define SWITCH_INPUT_TYPE                   50

enum notification_type {
    MOTCTRL_NTF_EMPTY, MOTCTRL_NTF_CONFIG_CHANGED
};

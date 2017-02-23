/*
 * rem_16mt_struct.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#define REM_16MT_SENSOR_RESOLUTION      (1<<16)

#define REM_16MT_CTRL_RESET         0x00
#define REM_16MT_CONF_DIR           0x55
#define REM_16MT_CONF_NULL          0x56
#define REM_16MT_CONF_PRESET        0x57
#define REM_16MT_CONF_STPRESET      0x50
#define REM_16MT_CONF_MTPRESET      0x59
#define REM_16MT_CONF_FILTER        0x5B
#define REM_16MT_CALIB_TBL_SIZE     0x3D
#define REM_16MT_CALIB_TBL_POINT    0x3E
#define REM_16MT_CTRL_SAVE          0x1C


/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    int filter;                 /**< filter parameter for rem_16mtt encoder */
} REM_16MTConfig;

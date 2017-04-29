/*
 * rem_16mt_struct.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#define REM_16MT_SENSOR_RESOLUTION      (1<<16)

#define REM_16MT_CTRL_RESET         0x00    /**< Software reset */
#define REM_16MT_CONF_DIR           0x55    /**< Polarity 0=CW, 1=CCW */
#define REM_16MT_CONF_NULL          0x56    /**< Reset the position to 0 */
#define REM_16MT_CONF_PRESET        0x57    /**< Reset both multiturn and single turn position to a new value */
#define REM_16MT_CONF_STPRESET      0x50    /**< Reset singleturn position to a new value */
#define REM_16MT_CONF_MTPRESET      0x59    /**< Reset multiturn position to a new value */
#define REM_16MT_CONF_FILTER        0x5B    /**< Filter setting. 0x00 to disable. 0x02 to 0x09 to enable. 0x09 is the strongest. */
#define REM_16MT_CALIB_TBL_SIZE     0x3D    /**< Start a calibration with the given table size (4,8,16,...,256)
                                                 This reset the offset and the polarity to 0
                                                 It is recommended to use a strong filter (0x09) during calibration.
                                                 Wait at least 200 ms before starting the first calibration point.  */
#define REM_16MT_CALIB_TBL_POINT    0x3E    /**< Setup a calibration point, the magnet should be positioned to the corresponding angle before.
                                                 The first calibration point is 0 then the angle should be increased in CW direction for each point [0 to (table size - 1)].
                                                 For example for a 4 points calibration the first point is 0°, then 90°, 180° and 270°.
                                                 After sending the last point the table is saved to the flash memory. */
#define REM_16MT_CTRL_SAVE          0x1C    /**< Save the calibration and configuration data to flash. It is recommended to send a reset command after saving. */


/**
 * @brief Structure type to define the REM16 Sensor configuration.
 */
typedef struct {
    int filter;                 /**< filter parameter for REM 16MT sensor. 0x00 to disable. 0x02 to 0x09 to enable. 0x09 is the strongest.*/
} REM_16MTConfig;

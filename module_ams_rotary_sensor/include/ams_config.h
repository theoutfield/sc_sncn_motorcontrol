/*
 * ams_config.h
 *
 *  Created on: Nov 20, 2015
 *      Author: support@synapticon.com
 */

#pragma once

#define AMS_INIT_SETTINGS1  1//5    // Factory Setting 1
                                    // NOISESET 0
                                    // DIR      0   (CW)
                                    // UVW_ABI  0
                                    // DAECDIS  0
                                    // ABIBIN   0
                                    // Dataselect 0
                                    // PWMon    0

#define AMS_INIT_SETTINGS2  4    //UVWPP     001 (5)
                                 //HYS       0
                                 //ABIRES    0

#define ROTARY_SENSOR_RESOLUTION_BITS 14

#define ENABLE_INDEPENDANT_AQUISITION 0 //0 - disabled, 1 - enabled

#define SENSOR_PLACEMENT_OFFSET 2555

#define MAX_COUNT_TICKS_CW (1<<ROTARY_SENSOR_RESOLUTION_BITS) * 1000
#define MAX_COUNT_TICKS_CCW (1<<ROTARY_SENSOR_RESOLUTION_BITS) * 1000

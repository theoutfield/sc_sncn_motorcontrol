/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: support@synapticon.com
 */

#pragma once

#define SPI_MASTER_MODE 1
//#define DEFAULT_SPI_CLOCK_DIV 10        // (100MHz / (10) = 10 MHz [100MHz ref clock]
#define DEFAULT_SPI_CLOCK_DIV 200        // (250MHz / (50) = 5 MHz [250MHz ref clock]

#define AMS_SENSOR_EXECUTING_TIME 125       //50us at 250MHz
#define AMS_SENSOR_SAVING_TIME 50           //20us at 250 MHz

#include <spi_master.h>
#include <xclib.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct sensor_spi_interface
{
    spi_master_interface spi_interface;
    out port slave_select;
} sensor_spi_interface;

typedef struct ams_config_params
{
    char settings1;
    char settings2;
    char enable_aquisition;
    int sensor_placement_offset;
    int resolution_bits;
    int max_count_ticks_cw;
    int max_count_ticks_ccw;
} ams_config_params_t;

interface AMS{
    int get_angle_electrical(void);
    {int, int} get_absolute_position_multiturn(void);
    int get_absolute_position_singleturn(void);
    int get_velocity(void);
    void configure(ams_config_params_t config_params);//ToDo: extend this configuration. Now configures whether reading measuremnts on multiturn position request is required.
};

ams_config_params_t set_configuration(void);

//#define ROTARY_SENSOR_MAX_ANGLE   16384
//#define ROTARY_SENSOR_RESOLUTION_BITS 14

//volatile registers
#define ADDR_ERRFL      0x0001
#define ADDR_PROG       0x0003
#define ADDR_DIAAGC     0x3FFC
#define ADDR_MAG        0x3FFD
#define ADDR_ANGLEUNC   0x3FFE
#define ADDR_ANGLECOM   0x3FFF

//non-volatile registers
#define ADDR_ZPOSM      0x0016
#define ADDR_ZPOSL      0x0017
#define ADDR_SETTINGS1  0x0018
#define ADDR_SETTINGS2  0x0019
#define ADDR_RED        0x001A

//COMMAND MASKS
#define WRITE_MASK 0xBFFF
#define READ_MASK 0x4000

//DATA MASKS
#define BITS_14_MASK    0x3FFF
#define BITS_12_MASK    0x0FFF
#define BITS_8_MASK     0x00FF
#define BITS_7_MASK     0x007F
#define BITS_6_MASK     0x003F
#define BITS_5_MASK     0x001F
#define BITS_3_MASK     0x0007

#define POLE_PAIRS_ZERO_MASK 0xFFF8
#define POLE_PAIRS_SET_MASK  0x0007
#define ROTATION_SENSE_CW_MASK  0x
#define ROTATION_SENSE_CCW_MASK 0x

//RETURN VALUES
#define SUCCESS_WRITING  1
#define PARITY_ERROR    -1
#define ERROR_WRITING   -2

void initRotarySensorInterface(sensor_spi_interface &sensor_if);
int initRotarySensor(sensor_spi_interface &sensor_if, unsigned short settings1, unsigned short settings2, unsigned short offset);

//reading fx
//non-volatile regs
int readZeroPosition(sensor_spi_interface &sensor_if);
int readNumberPolePairs(sensor_spi_interface &sensor_if);
int readSettings1(sensor_spi_interface &sensor_if);
int readSettings2(sensor_spi_interface &sensor_if);
int readRedundancyReg(sensor_spi_interface &sensor_if);

//volatile regs
int readProgrammingReg(sensor_spi_interface &sensor_if);
int readCORDICMagnitude(sensor_spi_interface &sensor_if);
int readRotaryDiagnosticAndAutoGainControl(sensor_spi_interface &sensor_if);
int readRotarySensorError(sensor_spi_interface &sensor_if);
int readRotarySensorAngleWithoutCompensation(sensor_spi_interface &sensor_if);
int readRotarySensorAngleWithCompensation(sensor_spi_interface &sensor_if);

//writing fx
int writeSettings1(sensor_spi_interface &sensor_if, unsigned short data);
int writeSettings2(sensor_spi_interface &sensor_if, unsigned short data);
int writeZeroPosition(sensor_spi_interface &sensor_if, unsigned short data);
int writeNumberPolePairs(sensor_spi_interface &sensor_if, unsigned short data);

//server for commutation and position control applications
void ams_sensor_server(server interface AMS iAMS[n], unsigned n, sensor_spi_interface &sensor_if);

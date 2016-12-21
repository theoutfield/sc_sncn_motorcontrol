#pragma once

#include <stdint.h>
//#include <spi_master.h>
#include "refclk.h"
#include <position_feedback_service.h>
//#include <user_config.h>

#define IFM_TILE_USEC   USEC_STD // Number of ticks in a microsecond for IFM Tile

#define as5050a_USEC            IFM_TILE_USEC

#define REM_14_SENSOR      5

#define ERROR       0
#define SUCCESS     1

#define SPI_MASTER_MODE 1

#define as5050a_SENSOR_EXECUTING_TIME (as5050a_USEC)
#define as5050a_SENSOR_SAVING_TIME    (as5050a_USEC/2)

//typedef struct {
//    spi_master_interface spi_interface;
//    port ?slave_select;
//} SPIPorts;

typedef struct {

    int velocity_loop;          /**< Velcity loop time in microseconds */

    int max_ticks;              /**< The count is reset to 0 if greater than this */

} AS5050A_Config;


#define REM_14_MAX_RESOLUTION   16384


#define ADDR_PorOFF        0x3F22
#define ADDR_SoftReset     0x3C00
#define ADDR_MasterReset   0x33A5
#define ADDR_ClearEF       0x3380
#define ADDR_NOP           0x0000
#define ADDR_AGC           0x3FF8
#define ADDR_AngularData   0x3FFF
#define ADDR_ErrorStatus   0x335A
#define ADDR_SystemConfig  0x3F20

//COMMAND MASKS
#define WRITE_MASK 0x3FFF      //0011 1111 1111 1111
#define READ_MASK  0x4000      //0100 0000 0000 0000

//DATA MASKS

#define BITS_10_MASK     0xFFC    //0000 1111 1111 1100
#define BITS_14_MASK     0xFFFC   //1111 1111 1111 1100
#define BITS_exact_MASK  0x3FFF   //0011 1111 1111 1111
#define BITS_8_MASK      0x3FC    //0000 0011 1111 1100
#define BITS_6_MASK      0xFC     //0000 0000 1111 1100
#define BITS_1_MASK      0x0010   //0000 0000 0100 0000
#define BITS_Lo_MASK     0x8000   //1000 0000 0000 0000
#define BITS_Hi_MASK     0x4000   //0100 0000 0000 0000

//RETURN VALUES
#define SUCCESS_WRITING     1
#define PARITY_ERROR       -1
#define ERROR_WRITING      -2
#define SENSOR_NOT_READY   -3

void initspiPorts(SPIPorts &spi_ports);
uint8_t calcParity(unsigned short bitStream);
unsigned short add_EvenParity(unsigned short bitStream);
unsigned char check_EvenParity(unsigned short bitStream);
short ReadTransactionSPI(SPIPorts &spi_ports, unsigned short reg);
short WriteTransactionSPI(SPIPorts &spi_ports, unsigned short reg, unsigned short data);
int checkWOWBit(SPIPorts &spi_ports);
int readAngleValue(SPIPorts &spi_ports);
int readAutomaticGainControl(SPIPorts &spi_ports);
int porCellDeactivate(SPIPorts &spi_ports);
int initas5050a(SPIPorts &spi_ports);

void as5050a_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3]);

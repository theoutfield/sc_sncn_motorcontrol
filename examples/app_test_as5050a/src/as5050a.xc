/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <as5050a.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <timer.h>
#include <print.h>
//#include <mc_internal_constants.h>
#include <ctype.h>
#include <spi_master.h>

//extern char start_message[];

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initspi_ports(SPIPorts &spi_ports)
{
    spi_master_init(spi_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
    slave_deselect(spi_ports.slave_select); // Ensure slave select is in correct start state
}

/**
 * @return 1 if uneven parity, 0 even parity
 */
uint8_t calc_parity(unsigned short bitStream)
{
    uint8_t parity = 0;

    for(unsigned i = 1;i<16;i++){
        parity += ((bitStream >> i) & 1);           //count number of 1s
    }
    return parity % 2;                           //mod2 of the number of 1s
}
// 0011 1100 0000 0000
/*
 * Set the LSB [0] acording to the remaining
 * bitstream [15:1] even parity.
 *
 */
unsigned short addEvenParity(unsigned short bitStream){
     return (bitStream |= (calc_parity(bitStream) << 0));      //set parity bit to this
}

/*
 * Check if the parity bit [0] is right.
 * Returns 1 if true, 0 if false.
 *
 */
unsigned char checkEvenParity(unsigned short bitStream){
     return (calc_parity(bitStream) == ((bitStream >> 0) & 1));    //comparison the parity bit the the real one
}

short SPIReadTransaction(SPIPorts &spi_ports, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = reg <<1;
    reg = addEvenParity(reg);                   //parity

    slave_select(spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);     //send command
    spi_ports.spi_interface.mosi <: 0;

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(as5050a_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(spi_ports.slave_select);                   //on the sensor

    data_in = spi_master_in_short(spi_ports.spi_interface); //handle response

    slave_deselect(spi_ports.slave_select);                 //end transaction

    return data_in;

}

short SPIWriteTransaction(SPIPorts &spi_ports, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = reg << 1;
    reg = addEvenParity(reg);                   //parity

  //  data &= WRITE_MASK;                         //action
    data = data << 1;
    data = addEvenParity(data);                 //parity

    slave_select(spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);     //send command

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(as5050a_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(spi_ports.slave_select);                   //on the sensor

    spi_master_out_short(spi_ports.spi_interface, data);
    spi_ports.spi_interface.mosi <: 0;

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(as5050a_SENSOR_SAVING_TIME);                    //saving the data
    slave_select(spi_ports.slave_select);                   //on the reg

    data_in = spi_master_in_short(spi_ports.spi_interface); //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(spi_ports.slave_select);                 //end transaction

    return data_in;
}

/* Add check WOW value */

int checkWOWBit(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_ErrorStatus);

    if(checkEvenParity(data_in)){            //check right parity

        return ((data_in & BITS_1_MASK) >> 6);         //get WOW bit

    }else{

        return PARITY_ERROR;
    }
}

int readAngleValue(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    if (checkWOWBit(spi_ports) == 0x0)
    {

    data_in = SPIReadTransaction(spi_ports, ADDR_AngularData);
   // printstrln("sensor ready! read now..");
    }
    else
    {
        return SENSOR_NOT_READY;
    }
    if(checkEvenParity(data_in)){                     //check right parity

        return ((data_in & BITS_10_MASK ) >> 2);        //remove unused bits
      //  return (data_in & BITS_10_MASK);            //remove unused bits

    }else{

       // printstrln("parity error");
        return PARITY_ERROR;

    }
}

int readAutomaticGainControl(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_AGC);

    if(checkEvenParity(data_in)){                     //check right parity

     //   printstrln("correct parity");
        return ((data_in & BITS_6_MASK ) >> 2);        //remove unused bits

    }else{

        return PARITY_ERROR;

    }
}
int porCellDeactivate(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    unsigned short data = 0x5A;
    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(spi_ports, ADDR_PorOFF, data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) == data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }
}

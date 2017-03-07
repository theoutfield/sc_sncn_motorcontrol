/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <rem_14_service.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <timer.h>
#include <print.h>
#include <mc_internal_constants.h>

extern char start_message[];

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
    slave_deselect(*spi_ports.slave_select); // Ensure slave select is in correct start state
}

{unsigned short, unsigned short} transform_settings(PositionFeedbackConfig &config)
{
    unsigned short settings1 = 0, settings2 = 0;

    #if REM_14_SENSOR_TYPE == AS5147
    settings1 = (REM_14_WIDTH_INDEX_PULSE << 0);
    #else
    settings1 = (REM_14_FACTORY_SETTINGS << 0);
    #endif
    settings1 |= (config.rem_14_config.noise_setting << 1);
    if (config.polarity == SENSOR_POLARITY_INVERTED) {
        settings1 |= (1 << 2);
    }
    settings1 |= (REM_14_UVW_ABI << 3);
    settings1 |= (config.rem_14_config.dyn_angle_comp << 4);
    settings1 |= (REM_14_DATA_SELECT << 6);
    settings1 |= (REM_14_PWM_CONFIG << 7);

    settings2 = (config.pole_pairs-1) << 0;
    settings2 |= (config.rem_14_config.hysteresis << 3);
    settings2 |= (config.rem_14_config.abi_resolution << 5);

    return {settings1, settings2};
}

SensorError initRotarySensor(SPIPorts &spi_ports, PositionFeedbackConfig &config)
{
    int data_in;

    unsigned short settings1, settings2;

    {settings1, settings2} = transform_settings(config);

    data_in = writeSettings(spi_ports, config.ifm_usec, ADDR_SETTINGS1, settings1);

    if(data_in < 0){

       return data_in;
    }

    delay_ticks(1000*config.ifm_usec);

    data_in = writeSettings(spi_ports, config.ifm_usec, ADDR_SETTINGS2, settings2);

    if(data_in < 0){

       return data_in;
    }

    delay_ticks(1000*config.ifm_usec);

    data_in = writeZeroPosition(spi_ports, config.ifm_usec, config.offset);

    if(data_in < 0){

       return data_in;
    }

    return SENSOR_NO_ERROR;
}

/**
 * @return 1 if uneven parity, 0 even parity
 */
uint8_t calc_parity(unsigned short bitStream)
{
    uint8_t parity = 0;

    for(unsigned i = 0;i<15;i++){
        parity += ((bitStream >> i) & 1);           //count number of 1s
    }
    return parity % 2;                           //mod2 of the number of 1s
}

/*
 * Set the MSB [15] acording to the remaining
 * bitstream [14:0] even parity.
 *
 */
unsigned short addEvenParity(unsigned short bitStream){
     return (bitStream |= (calc_parity(bitStream) << 15));      //set parity bit to this
}

/*
 * Check if the parity bit [15] is right.
 * Returns 1 if true, 0 if false.
 *
 */
unsigned char checkEvenParity(unsigned short bitStream){
     return (calc_parity(bitStream) == ((bitStream >> 15) & 1));    //comparison the parity bit the the real one
}

short SPIReadTransaction(SPIPorts &spi_ports, UsecType ifm_usec, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(*spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);      //send command
    *spi_ports.spi_interface.mosi <: 0;

    slave_deselect(*spi_ports.slave_select);                 //pause for
    delay_ticks(ifm_usec/REM_14_EXECUTING_TIME);      //executing the command
    slave_select(*spi_ports.slave_select);                   //on the sensor

    data_in = spi_master_in_short(spi_ports.spi_interface);  //handle response

    slave_deselect(*spi_ports.slave_select);                 //end transaction

    return data_in;
}

short SPIWriteTransaction(SPIPorts &spi_ports, UsecType ifm_usec, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(*spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);      //send command

    slave_deselect(*spi_ports.slave_select);                 //pause for
    delay_ticks(ifm_usec/REM_14_EXECUTING_TIME);      //executing the command
    slave_select(*spi_ports.slave_select);                   //on the sensor

    spi_master_out_short(spi_ports.spi_interface, data);
    *spi_ports.spi_interface.mosi <: 0;

    slave_deselect(*spi_ports.slave_select);                 //pause for
    delay_ticks(ifm_usec/REM_14_SAVING_TIME);         //saving the data
    slave_select(*spi_ports.slave_select);                   //on the reg

    data_in = spi_master_in_short(spi_ports.spi_interface);  //handle response

    slave_deselect(*spi_ports.slave_select);                 //end transaction

    return data_in;
}


int readRedundancyReg(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in;

    data_in = SPIReadTransaction(spi_ports, ifm_usec,ADDR_RED);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_5_MASK);
    }else{

        return REM_14_PARITY_ERROR;
    }
}

int readProgrammingReg(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_PROG);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_7_MASK);

    }else{

        return REM_14_PARITY_ERROR;
    }
}

int readSettings1(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_SETTINGS1);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return REM_14_PARITY_ERROR;
    }
}

int readSettings2(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_SETTINGS2);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return REM_14_PARITY_ERROR;
    }
}

int readZeroPosition(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0, data_in_tmp = 0;

    data_in_tmp = SPIReadTransaction(spi_ports, ifm_usec, ADDR_ZPOSM);   //register address (MSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

         data_in_tmp  = (data_in_tmp & BITS_8_MASK);                //masking
         data_in = (data_in_tmp << 6);                              //saving MSB
         data_in_tmp = 0;

    }else{

        return REM_14_PARITY_ERROR;
    }

    data_in_tmp = SPIReadTransaction(spi_ports, ifm_usec, ADDR_ZPOSL);   //register address (LSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

             data_in_tmp  = (data_in_tmp & BITS_6_MASK);            //remove unused bits
             data_in += data_in_tmp;                                //add LSB

             return data_in;

        }else{

            return REM_14_PARITY_ERROR;
        }
}

int readCORDICMagnitude(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_MAG);  //register address

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return REM_14_PARITY_ERROR;
    }
}


int readRotaryDiagnosticAndAutoGainControl(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_DIAAGC);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_12_MASK);        //remove unused bits

    }else{

        return REM_14_PARITY_ERROR;
    }
}

int readRotarySensorError(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_ERRFL);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_3_MASK);         //remove unused bits

    }else{

        return REM_14_PARITY_ERROR;
    }
}


{ unsigned int, unsigned int } readRotarySensorAngleWithoutCompensation(SPIPorts &spi_ports, UsecType ifm_usec){

   unsigned short data_in = 0;

   data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_ANGLEUNC);

   if(checkEvenParity(data_in)){             //check right parity

       return { (data_in & BITS_14_MASK), SENSOR_NO_ERROR };         //remove unused bits

   }else{

       return { (data_in & BITS_14_MASK), SENSOR_CHECKSUM_ERROR };
   }
}


{ unsigned int, unsigned int } readRotarySensorAngleWithCompensation(SPIPorts &spi_ports, UsecType ifm_usec){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ifm_usec, ADDR_ANGLECOM);

    if(checkEvenParity(data_in)){            //check right parity

        return { (data_in & BITS_14_MASK), SENSOR_NO_ERROR }; //remove unused bits

    } else {

        return { (data_in & BITS_14_MASK), SENSOR_CHECKSUM_ERROR };
    }
}

int readNumberPolePairs(SPIPorts &spi_ports, UsecType ifm_usec){

    int data_in = 0;

    data_in = readSettings2(spi_ports, ifm_usec);                //read current settings

    if(data_in < 0){
        return data_in;
    }

    data_in &= POLE_PAIRS_SET_MASK;                             //clean pole pairs bits
    data_in += 1;                                             //add 1 because of REM_14 sensor convention

    return data_in;
}


int writeSettings(SPIPorts &spi_ports, UsecType ifm_usec, unsigned short address, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(spi_ports, ifm_usec, address, data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) == data){

            return REM_14_SUCCESS_WRITING;
        }else{

            return REM_14_ERROR_WRITING;
        }
    }else{

        return REM_14_PARITY_ERROR;
    }
}

int writeZeroPosition(SPIPorts &spi_ports, UsecType ifm_usec, unsigned short data){

    unsigned short data_in = 0, msb_data = 0, lsb_data = 0;

    msb_data = (data >> 6) & BITS_8_MASK;

    data_in = SPIWriteTransaction(spi_ports, ifm_usec, ADDR_ZPOSM, msb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) != msb_data){

            return REM_14_ERROR_WRITING;
        }
    }else{

        return REM_14_PARITY_ERROR;
    }

    lsb_data = data & BITS_6_MASK;
    data_in = SPIWriteTransaction(spi_ports, ifm_usec, ADDR_ZPOSL, lsb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) == lsb_data){

            return REM_14_SUCCESS_WRITING;
        }else{

            return REM_14_ERROR_WRITING;
        }
    }else{

            return REM_14_PARITY_ERROR;
        }
}

int writeNumberPolePairs(SPIPorts &spi_ports, UsecType ifm_usec, unsigned short data){

    int data_in = 0;

    data -= 1;                                              //substract 1 because of REM_14 sensor convention
    data &= POLE_PAIRS_SET_MASK;                            //mask pole pairs bits

    data_in = readSettings2(spi_ports, ifm_usec);           //read current settings

    if(data_in < 0){                                        //something went wrong
        return data_in;
    }

    data_in &= POLE_PAIRS_ZERO_MASK;                        //clean pole pairs bits
    data_in |= data;                                        //add new pole pairs bits
    data_in = writeSettings(spi_ports, ifm_usec, ADDR_SETTINGS2, data_in); //write new settings

    return data_in;
}

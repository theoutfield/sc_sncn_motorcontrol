/*
 * rotary_sensor.xc
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#include <rotary_sensor.h>



static char rotarySensorInitialized = 0;

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initRotarySensorInterface(sensor_spi_interface &sensorInterface){

    if(rotarySensorInitialized != 1){

        spi_master_init(sensorInterface.spi_interface, DEFAULT_SPI_CLOCK_DIV);
        slave_deselect(sensorInterface.slave_select); // Ensure slave select is in correct start state
        rotarySensorInitialized = 1;

    }
}

int initRotarySensor(sensor_spi_interface &rotarySensorInterface, unsigned short settings1, unsigned short settings2, unsigned short offset){

    int data_in;


    initRotarySensorInterface(rotarySensorInterface);

    data_in = writeSettings1(rotarySensorInterface, settings1);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeSettings2(rotarySensorInterface, settings2);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeZeroPosition(rotarySensorInterface, offset);

    if(data_in < 0){

       return data_in;
    }

    return SUCCESS_WRITING;

}


/*
 * Set the MSB [15] acording to the remaining
 * bitstream [14:0] even parity.
 *
 */
unsigned short addEvenParity(unsigned short bitStream){

    int8_t parity = 0;

    for(unsigned i = 0;i<15;i++){
        parity += ((bitStream >> i) & 1);       //count number of 1s
    }
     parity = parity % 2;                       //number of 1 mod2

     return (bitStream |= (parity << 15));      //set parity bit to this

}

/*
 * Check if the parity bit [15] is right.
 * Returns 1 if true, 0 if false.
 *
 */
unsigned char checkEvenParity(unsigned short bitStream){

    int8_t parity = 0;

    for(unsigned i = 0;i<15;i++){
        parity += ((bitStream >> i) & 1);           //count number of 1s
    }
     parity = parity % 2;                           //mod2 of the number of 1s

     return (parity == ((bitStream >> 15) & 1));    //comparison the parity bit the the real one
}

short SPIReadTransaction(sensor_spi_interface &sensorInterface, unsigned short reg){

    timer t;
    unsigned ts;
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(sensorInterface.slave_select);                       //start transaction

    spi_master_out_short(sensorInterface.spi_interface, reg);          //send command
    sensorInterface.spi_interface.mosi <: 0;

    slave_deselect(sensorInterface.slave_select);                     //pause
    t:>ts;                                      //for executing
    t when timerafter(ts+50):>void;            //the command
    slave_select(sensorInterface.slave_select);                       //on the sensor

    data_in = spi_master_in_short(sensorInterface.spi_interface);      //handle response

    slave_deselect(sensorInterface.slave_select);                     //end transaction

    return data_in;
}

short SPIWriteTransaction(sensor_spi_interface &sensorInterface, unsigned short reg, unsigned short data){

    timer t;
    unsigned ts;
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(sensorInterface.slave_select);                       //start transaction

    spi_master_out_short(sensorInterface.spi_interface, reg);          //send command

    slave_deselect(sensorInterface.slave_select);                       //pause
    t:>ts;                                                              //for executing
    t when timerafter(ts+AMS_SENSOR_EXECUTING_TIME):>void;             //the command
    slave_select(sensorInterface.slave_select);                         //on the sensor

    spi_master_out_short(sensorInterface.spi_interface, data);
    sensorInterface.spi_interface.mosi <: 0;

    slave_deselect(sensorInterface.slave_select);                       //pause
    t:>ts;                                                              //for saving
    t when timerafter(ts+AMS_SENSOR_SAVING_TIME):>void;                //the data
    slave_select(sensorInterface.slave_select);                         //on the reg

    data_in = spi_master_in_short(sensorInterface.spi_interface);       //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(sensorInterface.slave_select);                       //end transaction

    return data_in;
}


int readRedundancyReg(sensor_spi_interface &sensorInterface){

    unsigned short data_in;

    data_in = SPIReadTransaction(sensorInterface,ADDR_RED);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_5_MASK);
    }else{

        return PARITY_ERROR;
    }
}



int readProgrammingReg(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_PROG);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_7_MASK);

    }else{

        return PARITY_ERROR;
    }
}

int readSettings1(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_SETTINGS1);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readSettings2(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_SETTINGS2);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readZeroPosition(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0, data_in_tmp = 0;

    data_in_tmp = SPIReadTransaction(sensorInterface, ADDR_ZPOSM);   //register address (MSB)

    if(checkEvenParity(data_in_tmp)==1){                            //check right parity

         data_in_tmp  = (data_in_tmp & BITS_8_MASK);                //masking
         data_in = (data_in_tmp << 6);                              //saving MSB
         data_in_tmp = 0;

    }else{

        return PARITY_ERROR;
    }

    data_in_tmp = SPIReadTransaction(sensorInterface, ADDR_ZPOSL);   //register address (LSB)

    if(checkEvenParity(data_in_tmp)==1){                            //check right parity

             data_in_tmp  = (data_in_tmp & BITS_6_MASK);            //remove unused bits
             data_in += data_in_tmp;                                //add LSB

             return data_in;

        }else{

            return PARITY_ERROR;
        }
}

int readCORDICMagnitude(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_MAG);  //register address

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}


int readRotaryDiagnosticAndAutoGainControl(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_DIAAGC);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_12_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorError(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_ERRFL);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_3_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorAngleWithoutCompensation(sensor_spi_interface &sensorInterface){

   unsigned short data_in = 0;

   data_in = SPIReadTransaction(sensorInterface, ADDR_ANGLEUNC);

   if(checkEvenParity(data_in)==1){             //check right parity

       return (data_in & BITS_14_MASK);         //remove unused bits

   }else{

       return PARITY_ERROR;
   }
}


int readRotarySensorAngleWithCompensation(sensor_spi_interface &sensorInterface){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(sensorInterface, ADDR_ANGLECOM);

    if(checkEvenParity(data_in)==1){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readNumberPolePairs(sensor_spi_interface &sensorInterface){

    int data_in = 0;

    data_in = readSettings2(sensorInterface);                //read current settings

    if(data_in < 0){

        return data_in;
    }

    data_in &= POLE_PAIRS_SET_MASK;                             //clean pole pairs bits
    data_in += 1;                                             //add 1 because of AMS sensor convention

    return data_in;
}


int writeSettings1(sensor_spi_interface &sensorInterface, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(sensorInterface, ADDR_SETTINGS1, data);

    if(checkEvenParity(data_in)==1){            //check right parity

        if((data_in & BITS_8_MASK) == data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }
}

int writeSettings2(sensor_spi_interface &sensorInterface, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(sensorInterface, ADDR_SETTINGS2, data);

    if(checkEvenParity(data_in) == 1){            //check right parity

        if((data_in & BITS_8_MASK) == data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }
}

int writeZeroPosition(sensor_spi_interface &sensorInterface, unsigned short data){

    unsigned short data_in = 0, msb_data = 0, lsb_data = 0;

    msb_data = (data >> 6) & BITS_8_MASK;

    data_in = SPIWriteTransaction(sensorInterface, ADDR_ZPOSM, msb_data);

    if(checkEvenParity(data_in)==1){            //check right parity

        if((data_in & BITS_8_MASK) != msb_data){

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }

    lsb_data = data & BITS_6_MASK;
    data_in = SPIWriteTransaction(sensorInterface, ADDR_ZPOSL, lsb_data);

    if(checkEvenParity(data_in)==1){            //check right parity

        if((data_in & BITS_8_MASK) == lsb_data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

            return PARITY_ERROR;
        }
}

int writeNumberPolePairs(sensor_spi_interface &sensorInterface, unsigned short data){

    int data_in = 0;

    data -= 1;                                              //substract 1 because of AMS sensor convention
    data &= POLE_PAIRS_SET_MASK;                            //mask pole pairs bits

    data_in = readSettings2(sensorInterface);                     //read current settings

    if(data_in < 0){                                        //something went wrong

        return data_in;
    }

    data_in &= POLE_PAIRS_ZERO_MASK;                             //clean pole pairs bits
    data_in |= data;                                        //add new pole pairs bits
    data_in = writeSettings2(sensorInterface, data_in);           //write new settings

    return data_in;
}

//void positionFeedbackServer()


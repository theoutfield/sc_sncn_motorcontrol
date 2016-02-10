/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <ams_service.h>
#include <stdio.h>
#include <stdlib.h>
#include <timer.h>
#include <print.h>

static char rotarySensorInitialized = 0;

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initams_ports(AMSPorts &ams_ports)
{
    if (rotarySensorInitialized != 1){

        spi_master_init(ams_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
        slave_deselect(ams_ports.slave_select); // Ensure slave select is in correct start state
        rotarySensorInitialized = 1;
    }
}

{unsigned short, unsigned short} transform_settings(AMSConfig config)
{
    unsigned short settings1 = 0, settings2 = 0;

    #if AMS_SENSOR_TYPE == AS5147
    settings1 = (config.width_index_pulse << 0);
    #else
    settings1 = (config.factory_settings << 0);
    #endif
    settings1 |= (config.noise_setting << 1);
    settings1 |= (config.direction << 2);
    settings1 |= (config.uvw_abi << 3);
    settings1 |= (config.dyn_angle_comp << 4);
    settings1 |= (config.data_select << 6);
    settings1 |= (config.pwm_on << 7);

    settings2 = (config.pole_pairs-1) << 0;
    settings2 |= (config.hysteresis << 3);
    settings2 |= (config.abi_resolution << 5);

    return {settings1, settings2};
}

int initRotarySensor(AMSPorts &ams_ports, AMSConfig config)
{
    int data_in;

    unsigned short settings1, settings2;

    {settings1, settings2} = transform_settings(config);

    initams_ports(ams_ports);

    data_in = writeSettings1(ams_ports, settings1);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeSettings2(ams_ports, settings2);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeZeroPosition(ams_ports, config.offset);

    if(data_in < 0){

       return data_in;
    }

    return SUCCESS_WRITING;
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

short SPIReadTransaction(AMSPorts &ams_ports, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(ams_ports.slave_select);                   //start transaction

    spi_master_out_short(ams_ports.spi_interface, reg);     //send command
    ams_ports.spi_interface.mosi <: 0;

    slave_deselect(ams_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(ams_ports.slave_select);                   //on the sensor

    data_in = spi_master_in_short(ams_ports.spi_interface); //handle response

    slave_deselect(ams_ports.slave_select);                 //end transaction

    return data_in;
}

short SPIWriteTransaction(AMSPorts &ams_ports, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(ams_ports.slave_select);                   //start transaction

    spi_master_out_short(ams_ports.spi_interface, reg);     //send command

    slave_deselect(ams_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(ams_ports.slave_select);                   //on the sensor

    spi_master_out_short(ams_ports.spi_interface, data);
    ams_ports.spi_interface.mosi <: 0;

    slave_deselect(ams_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_SAVING_TIME);                    //saving the data
    slave_select(ams_ports.slave_select);                   //on the reg

    data_in = spi_master_in_short(ams_ports.spi_interface); //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(ams_ports.slave_select);                 //end transaction

    return data_in;
}


int readRedundancyReg(AMSPorts &ams_ports){

    unsigned short data_in;

    data_in = SPIReadTransaction(ams_ports,ADDR_RED);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_5_MASK);
    }else{

        return PARITY_ERROR;
    }
}

int readProgrammingReg(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_PROG);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_7_MASK);

    }else{

        return PARITY_ERROR;
    }
}

int readSettings1(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_SETTINGS1);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readSettings2(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_SETTINGS2);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readZeroPosition(AMSPorts &ams_ports){

    unsigned short data_in = 0, data_in_tmp = 0;

    data_in_tmp = SPIReadTransaction(ams_ports, ADDR_ZPOSM);   //register address (MSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

         data_in_tmp  = (data_in_tmp & BITS_8_MASK);                //masking
         data_in = (data_in_tmp << 6);                              //saving MSB
         data_in_tmp = 0;

    }else{

        return PARITY_ERROR;
    }

    data_in_tmp = SPIReadTransaction(ams_ports, ADDR_ZPOSL);   //register address (LSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

             data_in_tmp  = (data_in_tmp & BITS_6_MASK);            //remove unused bits
             data_in += data_in_tmp;                                //add LSB

             return data_in;

        }else{

            return PARITY_ERROR;
        }
}

int readCORDICMagnitude(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_MAG);  //register address

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}


int readRotaryDiagnosticAndAutoGainControl(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_DIAAGC);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_12_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorError(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_ERRFL);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_3_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorAngleWithoutCompensation(AMSPorts &ams_ports){

   unsigned short data_in = 0;

   data_in = SPIReadTransaction(ams_ports, ADDR_ANGLEUNC);

   if(checkEvenParity(data_in)){             //check right parity

       return (data_in & BITS_14_MASK);         //remove unused bits

   }else{

       return PARITY_ERROR;
   }
}


int readRotarySensorAngleWithCompensation(AMSPorts &ams_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(ams_ports, ADDR_ANGLECOM);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    } else {

        return PARITY_ERROR;
    }
}

int readNumberPolePairs(AMSPorts &ams_ports){

    int data_in = 0;

    data_in = readSettings2(ams_ports);                //read current settings

    if(data_in < 0){
        return data_in;
    }

    data_in &= POLE_PAIRS_SET_MASK;                             //clean pole pairs bits
    data_in += 1;                                             //add 1 because of AMS sensor convention

    return data_in;
}


int writeSettings1(AMSPorts &ams_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(ams_ports, ADDR_SETTINGS1, data);

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

int writeSettings2(AMSPorts &ams_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(ams_ports, ADDR_SETTINGS2, data);

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

int writeZeroPosition(AMSPorts &ams_ports, unsigned short data){

    unsigned short data_in = 0, msb_data = 0, lsb_data = 0;

    msb_data = (data >> 6) & BITS_8_MASK;

    data_in = SPIWriteTransaction(ams_ports, ADDR_ZPOSM, msb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) != msb_data){

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }

    lsb_data = data & BITS_6_MASK;
    data_in = SPIWriteTransaction(ams_ports, ADDR_ZPOSL, lsb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) == lsb_data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

            return PARITY_ERROR;
        }
}

int writeNumberPolePairs(AMSPorts &ams_ports, unsigned short data){

    int data_in = 0;

    data -= 1;                                              //substract 1 because of AMS sensor convention
    data &= POLE_PAIRS_SET_MASK;                            //mask pole pairs bits

    data_in = readSettings2(ams_ports);                     //read current settings

    if(data_in < 0){                                        //something went wrong
        return data_in;
    }

    data_in &= POLE_PAIRS_ZERO_MASK;                             //clean pole pairs bits
    data_in |= data;                                        //add new pole pairs bits
    data_in = writeSettings2(ams_ports, data_in);           //write new settings

    return data_in;
}

static inline void multiturn(int &count, int last_position, int position, int ticks_per_turn) {
        int difference = position - last_position;
        if (difference >= ticks_per_turn/2)
            count = count + difference - ticks_per_turn;
        else if (-difference >= ticks_per_turn/2)
            count = count + difference + ticks_per_turn;
        else
            count += difference;
}

int check_ams_config(AMSConfig &ams_config) {
    if(ams_config.direction < 0  || ams_config.direction > 1){
        printstrln("Wrong AMS configuration: wrong direction");
        return ERROR;
    }
    if( AMS_USEC <= 0 ){
        printstrln("Wrong AMS configuration: wrong AMS_USEC value");
        return ERROR;
    }
    if(ams_config.cache_time < 0){
        printstrln("Wrong AMS configuration: wrong timeout");
        return ERROR;
    }
    if(ams_config.pole_pairs < 1){
        printstrln("Wrong AMS configuration: wrong pole-pairs");
        return ERROR;
    }
    return SUCCESS;
}

[[combinable]]
 void ams_service(AMSPorts &ams_ports, AMSConfig & ams_config, interface AMSInterface server i_ams[5])
{
    //Set freq to 250MHz (always needed for velocity calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    if(check_ams_config(ams_config) == ERROR){
        printstrln("Error while checking the AMS sensor configuration");
        return;
    }
    if (initRotarySensor(ams_ports,  ams_config) != SUCCESS_WRITING) {
        printstrln("Error with SPI AMS sensor");
        return;
    }

    printstr(">>   SOMANET AMS SENSOR SERVICE STARTING...\n");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << ams_config.resolution_bits);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = ams_config.velocity_loop * AMS_USEC; //velocity loop time in clock ticks
    int velocity_factor = 60000000/ams_config.velocity_loop;
    //position
    unsigned int last_position = 0;
    int count = 0;
    int calib_flag = 0;
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_ams_read = 0;


    //main loop
    while (1) {
        select {
        //send electrical angle for commutation
        case i_ams[int i].get_ams_angle() -> unsigned int angle:
                if (calib_flag == 0) {
                    t :> time;
                    if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                        angle = readRotarySensorAngleWithCompensation(ams_ports);
                        t :> last_ams_read;
                        multiturn(count, last_position, angle, ticks_per_turn);
                        last_position = angle;
                    } else
                        angle = last_position;
                    if (ams_config.resolution_bits > 12)
                        angle = (ams_config.pole_pairs * (angle >> (ams_config.resolution_bits-12)) ) & 4095;
                    else
                        angle = (ams_config.pole_pairs * (angle << (12-ams_config.resolution_bits)) ) & 4095;
                } else
                    angle = 0;
                break;

        //send multiturn count and position
        case i_ams[int i].get_ams_position() -> { int out_count, unsigned int position }:
                t :> time;
                if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                    position = readRotarySensorAngleWithCompensation(ams_ports);
                    t :> last_ams_read;
                    multiturn(count, last_position, position, ticks_per_turn);
                    last_position = position;
                } else
                    position = last_position;
                //count reset
                if (count >= ams_config.max_ticks || count < -ams_config.max_ticks)
                    count = 0;
                out_count = count;
                break;

        //send position
        case i_ams[int i].get_ams_real_position() -> unsigned int position:
                position = readRotarySensorAngleWithoutCompensation(ams_ports);
                break;

        //send velocity
        case i_ams[int i].get_ams_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //receive new ams_config
        case i_ams[int i].set_ams_config(AMSConfig in_config):
                //update variables which depend on ams_config
                if (ams_config.direction != in_config.direction)
                    initRotarySensor(ams_ports,  in_config);
                else if (ams_config.offset != in_config.offset)
                    writeZeroPosition(ams_ports, in_config.offset);
                ams_config = in_config;
                ticks_per_turn = (1 << ams_config.resolution_bits);
                crossover = ticks_per_turn - ticks_per_turn/10;
                velocity_loop = ams_config.velocity_loop * AMS_USEC;
                velocity_factor = 60000000/ams_config.velocity_loop;

                break;

        //send ams_config
        case i_ams[int i].get_ams_config() -> AMSConfig out_config:
                out_config = ams_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_ams[int i].reset_ams_position(int new_count):
                last_position = readRotarySensorAngleWithoutCompensation(ams_ports);
                t :> last_ams_read;
                count = new_count;
                break;

        //receive the new elecrical angle to set the offset accordingly
        case i_ams[int i].reset_ams_angle(unsigned int new_angle) -> unsigned int out_offset:
                writeZeroPosition(ams_ports, 0);
                int position = readRotarySensorAngleWithoutCompensation(ams_ports);
                if (ams_config.resolution_bits > 12)
                    out_offset = (ticks_per_turn - ((new_angle << (ams_config.resolution_bits-12)) / ams_config.pole_pairs) + position) & (ticks_per_turn-1);
                else
                    out_offset = (ticks_per_turn - ((new_angle >> (12-ams_config.resolution_bits)) / ams_config.pole_pairs) + position) & (ticks_per_turn-1);
                writeZeroPosition(ams_ports, out_offset);
                ams_config.offset = out_offset;
                break;

        //set the calib flag, the server will alway return 0 as electrical angle
        case i_ams[int i].set_ams_calib(int flag) -> unsigned int angle:
                if (flag == 0) {
                    angle = readRotarySensorAngleWithoutCompensation(ams_ports);
                    if (ams_config.resolution_bits > 12)
                        angle = (ams_config.pole_pairs * (angle >> (ams_config.resolution_bits-12)) ) & 4095;
                    else
                        angle = (ams_config.pole_pairs * (angle << (12-ams_config.resolution_bits)) ) & 4095;
                }
                calib_flag = flag;
                break;

        //compute velocity
        case t when timerafter(next_velocity_read) :> void:
            next_velocity_read += velocity_loop;
            int position;
            t :> time;
            if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                position = readRotarySensorAngleWithCompensation(ams_ports);
                t :> last_ams_read;
                multiturn(count, last_position, position, ticks_per_turn);
                last_position = position;
            }
            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
            velocity = (difference * velocity_factor) / ticks_per_turn;
            break;
        }
    }
}


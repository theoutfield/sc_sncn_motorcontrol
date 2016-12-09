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
    slave_deselect(spi_ports.slave_select); // Ensure slave select is in correct start state
}

{unsigned short, unsigned short} transform_settings(PositionFeedbackConfig &config)
{
    unsigned short settings1 = 0, settings2 = 0;

    #if REM_14_SENSOR_TYPE == AS5147
    settings1 = (config.rem_14_config.width_index_pulse << 0);
    #else
    settings1 = (config.rem_14_config.factory_settings << 0);
    #endif
    settings1 |= (config.rem_14_config.noise_setting << 1);
    if (config.polarity == REM_14_POLARITY_INVERTED) {
        settings1 |= (1 << 2);
    }
    settings1 |= (config.rem_14_config.uvw_abi << 3);
    settings1 |= (config.rem_14_config.dyn_angle_comp << 4);
    settings1 |= (config.rem_14_config.data_select << 6);
    settings1 |= (config.rem_14_config.pwm_on << 7);

    settings2 = (config.pole_pairs-1) << 0;
    settings2 |= (config.rem_14_config.hysteresis << 3);
    settings2 |= (config.rem_14_config.abi_resolution << 5);

    return {settings1, settings2};
}

int initRotarySensor(SPIPorts &spi_ports, PositionFeedbackConfig &config)
{
    int data_in;

    unsigned short settings1, settings2;

    {settings1, settings2} = transform_settings(config);

    initspi_ports(spi_ports);

    data_in = writeSettings1(spi_ports, settings1);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeSettings2(spi_ports, settings2);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeZeroPosition(spi_ports, config.offset);

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

short SPIReadTransaction(SPIPorts &spi_ports, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);     //send command
    spi_ports.spi_interface.mosi <: 0;

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(REM_14_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(spi_ports.slave_select);                   //on the sensor

    data_in = spi_master_in_short(spi_ports.spi_interface); //handle response

    slave_deselect(spi_ports.slave_select);                 //end transaction

    return data_in;
}

short SPIWriteTransaction(SPIPorts &spi_ports, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(spi_ports.slave_select);                   //start transaction

    spi_master_out_short(spi_ports.spi_interface, reg);     //send command

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(REM_14_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(spi_ports.slave_select);                   //on the sensor

    spi_master_out_short(spi_ports.spi_interface, data);
    spi_ports.spi_interface.mosi <: 0;

    slave_deselect(spi_ports.slave_select);                 //pause for
    delay_ticks(REM_14_SENSOR_SAVING_TIME);                    //saving the data
    slave_select(spi_ports.slave_select);                   //on the reg

    data_in = spi_master_in_short(spi_ports.spi_interface); //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(spi_ports.slave_select);                 //end transaction

    return data_in;
}


int readRedundancyReg(SPIPorts &spi_ports){

    unsigned short data_in;

    data_in = SPIReadTransaction(spi_ports,ADDR_RED);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_5_MASK);
    }else{

        return PARITY_ERROR;
    }
}

int readProgrammingReg(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_PROG);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_7_MASK);

    }else{

        return PARITY_ERROR;
    }
}

int readSettings1(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_SETTINGS1);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readSettings2(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_SETTINGS2);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readZeroPosition(SPIPorts &spi_ports){

    unsigned short data_in = 0, data_in_tmp = 0;

    data_in_tmp = SPIReadTransaction(spi_ports, ADDR_ZPOSM);   //register address (MSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

         data_in_tmp  = (data_in_tmp & BITS_8_MASK);                //masking
         data_in = (data_in_tmp << 6);                              //saving MSB
         data_in_tmp = 0;

    }else{

        return PARITY_ERROR;
    }

    data_in_tmp = SPIReadTransaction(spi_ports, ADDR_ZPOSL);   //register address (LSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

             data_in_tmp  = (data_in_tmp & BITS_6_MASK);            //remove unused bits
             data_in += data_in_tmp;                                //add LSB

             return data_in;

        }else{

            return PARITY_ERROR;
        }
}

int readCORDICMagnitude(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_MAG);  //register address

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}


int readRotaryDiagnosticAndAutoGainControl(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_DIAAGC);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_12_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorError(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_ERRFL);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_3_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorAngleWithoutCompensation(SPIPorts &spi_ports){

   unsigned short data_in = 0;

   data_in = SPIReadTransaction(spi_ports, ADDR_ANGLEUNC);

   if(checkEvenParity(data_in)){             //check right parity

       return (data_in & BITS_14_MASK);         //remove unused bits

   }else{

       return PARITY_ERROR;
   }
}


int readRotarySensorAngleWithCompensation(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(spi_ports, ADDR_ANGLECOM);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    } else {

        return PARITY_ERROR;
    }
}

int readNumberPolePairs(SPIPorts &spi_ports){

    int data_in = 0;

    data_in = readSettings2(spi_ports);                //read current settings

    if(data_in < 0){
        return data_in;
    }

    data_in &= POLE_PAIRS_SET_MASK;                             //clean pole pairs bits
    data_in += 1;                                             //add 1 because of REM_14 sensor convention

    return data_in;
}


int writeSettings1(SPIPorts &spi_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(spi_ports, ADDR_SETTINGS1, data);

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

int writeSettings2(SPIPorts &spi_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(spi_ports, ADDR_SETTINGS2, data);

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

int writeZeroPosition(SPIPorts &spi_ports, unsigned short data){

    unsigned short data_in = 0, msb_data = 0, lsb_data = 0;

    msb_data = (data >> 6) & BITS_8_MASK;

    data_in = SPIWriteTransaction(spi_ports, ADDR_ZPOSM, msb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) != msb_data){

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }

    lsb_data = data & BITS_6_MASK;
    data_in = SPIWriteTransaction(spi_ports, ADDR_ZPOSL, lsb_data);

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

int writeNumberPolePairs(SPIPorts &spi_ports, unsigned short data){

    int data_in = 0;

    data -= 1;                                              //substract 1 because of REM_14 sensor convention
    data &= POLE_PAIRS_SET_MASK;                            //mask pole pairs bits

    data_in = readSettings2(spi_ports);                     //read current settings

    if(data_in < 0){                                        //something went wrong
        return data_in;
    }

    data_in &= POLE_PAIRS_ZERO_MASK;                             //clean pole pairs bits
    data_in |= data;                                        //add new pole pairs bits
    data_in = writeSettings2(spi_ports, data_in);           //write new settings

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

void rem_14_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3])
{

    if (REM_14_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    position_feedback_config.offset &= (position_feedback_config.resolution-1);
    if (initRotarySensor(spi_ports,  position_feedback_config) != SUCCESS_WRITING) {
        printstrln("Error with SPI REM_14 sensor");
        position_feedback_config.sensor_type = 0;
        return;
    }

    printstr(start_message);
    printstrln("REM_14");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
    int velocity_loop = position_feedback_config.rem_14_config.velocity_loop * REM_14_USEC; //velocity loop time in clock ticks
    int velocity_factor = 60000000/position_feedback_config.rem_14_config.velocity_loop;
    //position
    unsigned int last_position = 0;
    int count = 0;
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_read = 0;
    unsigned int last_velocity_read= 0;

    int notification = MOTCTRL_NTF_EMPTY;

    int actual_velocity = 0;
    int actual_count = 0;
    unsigned int actual_position = 0;
    unsigned int actual_angle = 0;
    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;

    //first read
    last_position = readRotarySensorAngleWithoutCompensation(spi_ports);
    t :> last_read;

    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                t :> time;
                if (timeafter(time, last_read + position_feedback_config.rem_14_config.cache_time)) {
                    angle = readRotarySensorAngleWithCompensation(spi_ports);
                    t :> last_read;
                    multiturn(count, last_position, angle, position_feedback_config.resolution);
                    last_position = angle;
                } else
                    angle = last_position;
                angle = (position_feedback_config.pole_pairs * (angle >> 2) ) & 4095;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                t :> time;
                if (timeafter(time, last_read + position_feedback_config.rem_14_config.cache_time)) {
                    position = readRotarySensorAngleWithCompensation(spi_ports);
                    t :> last_read;
                    multiturn(count, last_position, position, position_feedback_config.resolution);
                    last_position = position;
                } else
                    position = last_position;
                //count reset
                if (count >= position_feedback_config.rem_14_config.max_ticks || count < -position_feedback_config.rem_14_config.max_ticks)
                    count = 0;
                out_count = count;
                break;

        //send position
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                position = readRotarySensorAngleWithoutCompensation(spi_ports);
                break;

        //send velocity
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //send ticks per turn
        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                out_ticks_per_turn = position_feedback_config.resolution;
                break;

        //receive new rem_14_config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                in_config.offset &= (in_config.resolution-1);
                //update variables which depend on rem_14_config
                if (position_feedback_config.polarity != in_config.polarity)
                    initRotarySensor(spi_ports,  in_config);
                else if (position_feedback_config.offset != in_config.offset)
                    writeZeroPosition(spi_ports, in_config.offset);
                position_feedback_config = in_config;
                crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
                velocity_loop = position_feedback_config.rem_14_config.velocity_loop * REM_14_USEC;
                velocity_factor = 60000000/position_feedback_config.rem_14_config.velocity_loop;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                break;

        //send rem_14_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_position_feedback[int i].set_position(int new_count):
                last_position = readRotarySensorAngleWithoutCompensation(spi_ports);
                t :> last_read;
                count = new_count;
                break;

        //receive the new elecrical angle to set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                writeZeroPosition(spi_ports, 0);
                int position = readRotarySensorAngleWithoutCompensation(spi_ports);
                out_offset = (position_feedback_config.resolution - ((new_angle << 2) / position_feedback_config.pole_pairs) + position) & (position_feedback_config.resolution-1);
                writeZeroPosition(spi_ports, out_offset);
                position_feedback_config.offset = out_offset;
                break;

        //execute command
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                break;

        case i_position_feedback[int i].exit():
                loop_flag = 0;
                continue;

        //compute velocity
        case t when timerafter(next_velocity_read) :> start_time:
            next_velocity_read += velocity_loop;
            int position, angle;
            t :> time;
            position = readRotarySensorAngleWithCompensation(spi_ports);
            t :> last_read;
            multiturn(count, last_position, position, position_feedback_config.resolution);
            last_position = position;

            angle = (position_feedback_config.pole_pairs * (position >> 2) ) & 4095;

            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
//            velocity = (difference * velocity_factor) / ticks_per_turn;
            velocity = (difference * (60000000/((int)(last_read-last_velocity_read)/REM_14_USEC))) / position_feedback_config.resolution;
            last_velocity_read = last_read;

            if (!isnull(i_shared_memory)) {
                if (position_feedback_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(angle, velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_angle = angle;
                    actual_position = position;
                } else if (position_feedback_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(angle);
                    actual_angle = angle;
                } else if (position_feedback_config.enable_push_service == PushPosition) {
                    i_shared_memory.write_velocity_position(velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_position = position;
                }
            }
            t :> end_time;

            measurement_time = (end_time-start_time)/USEC_FAST;

            //to prevent blocking
            if (timeafter(end_time, next_velocity_read))
                next_velocity_read = end_time + REM_14_USEC;
            break;
        }
    }
}


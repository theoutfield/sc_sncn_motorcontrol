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
#include <mc_internal_constants.h>



static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initposition_feedback_ports(PositionFeedbackPorts &position_feedback_ports)
{
    //static char rotarySensorInitialized = 0;

    //if (rotarySensorInitialized != 1){

        spi_master_init(position_feedback_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
        slave_deselect(position_feedback_ports.slave_select); // Ensure slave select is in correct start state
        //rotarySensorInitialized = 1;
    //}
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
    settings1 |= (config.polarity << 2);
    settings1 |= (config.uvw_abi << 3);
    settings1 |= (config.dyn_angle_comp << 4);
    settings1 |= (config.data_select << 6);
    settings1 |= (config.pwm_on << 7);

    settings2 = (config.pole_pairs-1) << 0;
    settings2 |= (config.hysteresis << 3);
    settings2 |= (config.abi_resolution << 5);

    return {settings1, settings2};
}

int initRotarySensor(PositionFeedbackPorts &position_feedback_ports, AMSConfig config)
{
    int data_in;

    unsigned short settings1, settings2;

    {settings1, settings2} = transform_settings(config);

    initposition_feedback_ports(position_feedback_ports);

    data_in = writeSettings1(position_feedback_ports, settings1);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeSettings2(position_feedback_ports, settings2);

    if(data_in < 0){

       return data_in;
    }

    delay_milliseconds(1);

    data_in = writeZeroPosition(position_feedback_ports, config.offset);

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

short SPIReadTransaction(PositionFeedbackPorts &position_feedback_ports, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(position_feedback_ports.slave_select);                   //start transaction

    spi_master_out_short(position_feedback_ports.spi_interface, reg);     //send command
    position_feedback_ports.spi_interface.mosi <: 0;

    slave_deselect(position_feedback_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(position_feedback_ports.slave_select);                   //on the sensor

    data_in = spi_master_in_short(position_feedback_ports.spi_interface); //handle response

    slave_deselect(position_feedback_ports.slave_select);                 //end transaction

    return data_in;
}

short SPIWriteTransaction(PositionFeedbackPorts &position_feedback_ports, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(position_feedback_ports.slave_select);                   //start transaction

    spi_master_out_short(position_feedback_ports.spi_interface, reg);     //send command

    slave_deselect(position_feedback_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_EXECUTING_TIME);                 //executing the command
    slave_select(position_feedback_ports.slave_select);                   //on the sensor

    spi_master_out_short(position_feedback_ports.spi_interface, data);
    position_feedback_ports.spi_interface.mosi <: 0;

    slave_deselect(position_feedback_ports.slave_select);                 //pause for
    delay_ticks(AMS_SENSOR_SAVING_TIME);                    //saving the data
    slave_select(position_feedback_ports.slave_select);                   //on the reg

    data_in = spi_master_in_short(position_feedback_ports.spi_interface); //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(position_feedback_ports.slave_select);                 //end transaction

    return data_in;
}


int readRedundancyReg(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in;

    data_in = SPIReadTransaction(position_feedback_ports,ADDR_RED);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_5_MASK);
    }else{

        return PARITY_ERROR;
    }
}

int readProgrammingReg(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_PROG);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_7_MASK);

    }else{

        return PARITY_ERROR;
    }
}

int readSettings1(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_SETTINGS1);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readSettings2(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_SETTINGS2);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_8_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readZeroPosition(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0, data_in_tmp = 0;

    data_in_tmp = SPIReadTransaction(position_feedback_ports, ADDR_ZPOSM);   //register address (MSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

         data_in_tmp  = (data_in_tmp & BITS_8_MASK);                //masking
         data_in = (data_in_tmp << 6);                              //saving MSB
         data_in_tmp = 0;

    }else{

        return PARITY_ERROR;
    }

    data_in_tmp = SPIReadTransaction(position_feedback_ports, ADDR_ZPOSL);   //register address (LSB)

    if(checkEvenParity(data_in_tmp)){                            //check right parity

             data_in_tmp  = (data_in_tmp & BITS_6_MASK);            //remove unused bits
             data_in += data_in_tmp;                                //add LSB

             return data_in;

        }else{

            return PARITY_ERROR;
        }
}

int readCORDICMagnitude(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_MAG);  //register address

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}


int readRotaryDiagnosticAndAutoGainControl(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_DIAAGC);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_12_MASK);        //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorError(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_ERRFL);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_3_MASK);         //remove unused bits

    }else{

        return PARITY_ERROR;
    }
}

int readRotarySensorAngleWithoutCompensation(PositionFeedbackPorts &position_feedback_ports){

   unsigned short data_in = 0;

   data_in = SPIReadTransaction(position_feedback_ports, ADDR_ANGLEUNC);

   if(checkEvenParity(data_in)){             //check right parity

       return (data_in & BITS_14_MASK);         //remove unused bits

   }else{

       return PARITY_ERROR;
   }
}


int readRotarySensorAngleWithCompensation(PositionFeedbackPorts &position_feedback_ports){

    unsigned short data_in = 0;

    data_in = SPIReadTransaction(position_feedback_ports, ADDR_ANGLECOM);

    if(checkEvenParity(data_in)){            //check right parity

        return (data_in & BITS_14_MASK);        //remove unused bits

    } else {

        return PARITY_ERROR;
    }
}

int readNumberPolePairs(PositionFeedbackPorts &position_feedback_ports){

    int data_in = 0;

    data_in = readSettings2(position_feedback_ports);                //read current settings

    if(data_in < 0){
        return data_in;
    }

    data_in &= POLE_PAIRS_SET_MASK;                             //clean pole pairs bits
    data_in += 1;                                             //add 1 because of AMS sensor convention

    return data_in;
}


int writeSettings1(PositionFeedbackPorts &position_feedback_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(position_feedback_ports, ADDR_SETTINGS1, data);

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

int writeSettings2(PositionFeedbackPorts &position_feedback_ports, unsigned short data){

    unsigned short data_in = 0;

    data &= BITS_8_MASK;
    data_in = SPIWriteTransaction(position_feedback_ports, ADDR_SETTINGS2, data);

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

int writeZeroPosition(PositionFeedbackPorts &position_feedback_ports, unsigned short data){

    unsigned short data_in = 0, msb_data = 0, lsb_data = 0;

    msb_data = (data >> 6) & BITS_8_MASK;

    data_in = SPIWriteTransaction(position_feedback_ports, ADDR_ZPOSM, msb_data);

    if(checkEvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) != msb_data){

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }

    lsb_data = data & BITS_6_MASK;
    data_in = SPIWriteTransaction(position_feedback_ports, ADDR_ZPOSL, lsb_data);

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

int writeNumberPolePairs(PositionFeedbackPorts &position_feedback_ports, unsigned short data){

    int data_in = 0;

    data -= 1;                                              //substract 1 because of AMS sensor convention
    data &= POLE_PAIRS_SET_MASK;                            //mask pole pairs bits

    data_in = readSettings2(position_feedback_ports);                     //read current settings

    if(data_in < 0){                                        //something went wrong
        return data_in;
    }

    data_in &= POLE_PAIRS_ZERO_MASK;                             //clean pole pairs bits
    data_in |= data;                                        //add new pole pairs bits
    data_in = writeSettings2(position_feedback_ports, data_in);           //write new settings

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
    if(ams_config.polarity < 0  || ams_config.polarity > 1){
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
 void ams_service(PositionFeedbackPorts &position_feedback_ports, AMSConfig & ams_config, client interface shared_memory_interface ?i_shared_memory,
         server interface PositionFeedbackInterface i_position_feedback[3])
{
    //Set freq to 250MHz (always needed for velocity calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    if(check_ams_config(ams_config) == ERROR){
        printstrln("Error while checking the AMS sensor configuration");
        return;
    }
    if (initRotarySensor(position_feedback_ports,  ams_config) != SUCCESS_WRITING) {
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
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_ams_read = 0;
    unsigned int last_velocity_read= 0;

    int notification = MOTCTRL_NTF_EMPTY;

    int actual_velocity = 0;
    int actual_count = 0;
    unsigned int actual_position = 0;
    unsigned int actual_angle = 0;
    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;

    //first read
    last_position = readRotarySensorAngleWithoutCompensation(position_feedback_ports);
    t :> last_ams_read;

    //main loop
    while (1) {
        select {
//        case i_position_feedback[int i].get_all() -> { int out_count, int out_velocity, unsigned int out_position, unsigned int out_angle, unsigned int out_time }:
//                out_count = actual_count;
//                out_velocity = actual_velocity;
//                out_position = actual_position;
//                out_angle = actual_angle;
//                out_time = measurement_time;
//                break;

        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                t :> time;
                if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                    angle = readRotarySensorAngleWithCompensation(position_feedback_ports);
                    t :> last_ams_read;
                    multiturn(count, last_position, angle, ticks_per_turn);
                    last_position = angle;
                } else
                    angle = last_position;
                if (ams_config.resolution_bits > 12)
                    angle = (ams_config.pole_pairs * (angle >> (ams_config.resolution_bits-12)) ) & 4095;
                else
                    angle = (ams_config.pole_pairs * (angle << (12-ams_config.resolution_bits)) ) & 4095;
                //out_velocity = velocity;
                //out_count = count;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                t :> time;
                if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                    position = readRotarySensorAngleWithCompensation(position_feedback_ports);
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
        case i_position_feedback[int i].get_real_position() -> { int count_out, unsigned int position_out, unsigned int status_out }:
                t :> time;
                if (timeafter(time, last_ams_read + ams_config.cache_time)) {
                    position_out = readRotarySensorAngleWithCompensation(position_feedback_ports);
                    t :> last_ams_read;
                    multiturn(count, last_position, position_out, ticks_per_turn);
                    last_position = position_out;
                } else
                    position_out = last_position;
                status_out = 0;
                count_out = count;
                break;

        //send velocity
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                out_ticks_per_turn = ticks_per_turn;
                break;

        //receive new ams_config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                ticks_per_turn = (1 << in_config.ams_config.resolution_bits);
                in_config.ams_config.offset &= (ticks_per_turn-1);
                //update variables which depend on ams_config
                if (ams_config.polarity != in_config.ams_config.polarity)
                    initRotarySensor(position_feedback_ports,  in_config.ams_config);
                else if (ams_config.offset != in_config.ams_config.offset)
                    writeZeroPosition(position_feedback_ports, in_config.ams_config.offset);
                ams_config = in_config.ams_config;
                crossover = ticks_per_turn - ticks_per_turn/10;
                velocity_loop = ams_config.velocity_loop * AMS_USEC;
                velocity_factor = 60000000/ams_config.velocity_loop;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                break;

        //send ams_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config.ams_config = ams_config;
                break;

        case i_position_feedback[int i].set_position(int in_count):
                break;

        case i_position_feedback[int i].set_angle(unsigned int in_angle) -> unsigned int out_offset:
                break;

        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int out_status:
            break;
//        //receive the new count to set and set the offset accordingly
//        case i_position_feedback[int i].reset_position(int new_count):
//                last_position = readRotarySensorAngleWithoutCompensation(position_feedback_ports);
//                t :> last_ams_read;
//                count = new_count;
//                break;
//
//        //receive the new elecrical angle to set the offset accordingly
//        case i_position_feedback[int i].reset_angle(unsigned int new_angle) -> unsigned int out_offset:
//                writeZeroPosition(position_feedback_ports, 0);
//                int position = readRotarySensorAngleWithoutCompensation(position_feedback_ports);
//                if (ams_config.resolution_bits > 12)
//                    out_offset = (ticks_per_turn - ((new_angle << (ams_config.resolution_bits-12)) / ams_config.pole_pairs) + position) & (ticks_per_turn-1);
//                else
//                    out_offset = (ticks_per_turn - ((new_angle >> (12-ams_config.resolution_bits)) / ams_config.pole_pairs) + position) & (ticks_per_turn-1);
//                writeZeroPosition(position_feedback_ports, out_offset);
//                ams_config.offset = out_offset;
//                break;

        //compute velocity
        case t when timerafter(next_velocity_read) :> start_time:
            next_velocity_read += velocity_loop;
            int position, angle;
            t :> time;
            position = readRotarySensorAngleWithCompensation(position_feedback_ports);
            t :> last_ams_read;
            multiturn(count, last_position, position, ticks_per_turn);
            last_position = position;

            if (ams_config.resolution_bits > 12)
                angle = (ams_config.pole_pairs * (position >> (ams_config.resolution_bits-12)) ) & 4095;
            else
                angle = (ams_config.pole_pairs * (position << (12-ams_config.resolution_bits)) ) & 4095;

            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
//            velocity = (difference * velocity_factor) / ticks_per_turn;
            velocity = (difference * (60000000/((int)(last_ams_read-last_velocity_read)/AMS_USEC))) / ticks_per_turn;
            last_velocity_read = last_ams_read;

            if (!isnull(i_shared_memory)) {
                if (ams_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(angle, velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_angle = angle;
                    actual_position = position;
                } else if (ams_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(angle);
                    actual_angle = angle;
                } else if (ams_config.enable_push_service == PushPosition) {
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
                next_velocity_read = end_time + AMS_USEC;
            break;
        }
    }
}


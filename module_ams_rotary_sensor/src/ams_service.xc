/*
 * rotary_sensor.xc
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#include <ams_service.h>
#include <ams_config.h>
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

AMSConfig set_configuration(void){
    AMSConfig ams_config;
    ams_config.settings1 = AMS_INIT_SETTINGS1;
    ams_config.settings2 = AMS_INIT_SETTINGS2;
    ams_config.enable_aquisition = ENABLE_INDEPENDENT_AQUISITION;
    ams_config.sensor_placement_offset = SENSOR_PLACEMENT_OFFSET;
    ams_config.resolution_bits = ROTARY_SENSOR_RESOLUTION_BITS;
    ams_config.max_count_ticks_cw = MAX_COUNT_TICKS_CW;
    ams_config.max_count_ticks_ccw = MAX_COUNT_TICKS_CCW;
    return ams_config;
}

void ams_sensor_server(server interface AMSInterface i_ams[n], unsigned n, sensor_spi_interface &sensor_if){

    int n_pole_pairs_ = 0;
    int sensor_resolution_ticks_ = 1;
    int sensor_resolution_bits_ = 0;
    int abs_pos_ = 0, abs_pos_previos_ = 0, abs_pos_old_vel_est_ = 0;
    int settings1_ = 0, settings2_ = 0;
    int offset_ = 0;
    int velocity_ = 0;
    int enable_aquisition_ = 0;
    int pos_multiturn_ = 0;
    int difference_old_ = 0;
    int max_count_ticks_cw_ = 0, max_count_ticks_ccw_ = 0;
    char sensor_initialized_ = 0;
    char measurement_taken_ = 0;
    int velocity_ticks_ = 0;
    timer t;
    unsigned int tmr;

    t :> tmr;
//    writeNumberPolePairs(sensor_if, 2);

    while(1)
    {
        select{
            case i_ams[int i].get_ams_angle_electrical(void) -> unsigned int angle_electrical:
                    if (sensor_initialized_ == 1){
                        if(settings1_ == 5){
                            abs_pos_ = sensor_resolution_ticks_ - readRotarySensorAngleWithCompensation(sensor_if);//readRotarySensorAngleWithoutCompensation(sensor_if);
                            measurement_taken_ = 1;
                        } else {
                            abs_pos_ = readRotarySensorAngleWithCompensation(sensor_if);//readRotarySensorAngleWithoutCompensation(sensor_if);
                            measurement_taken_ = 1;
                        }

                        if(sensor_resolution_bits_ > 12){
                            angle_electrical = (n_pole_pairs_ * abs_pos_ >> (sensor_resolution_bits_ - 12)) % 4096;
                            angle_electrical = abs_pos_;
                        }else{
                            angle_electrical = (n_pole_pairs_ * abs_pos_ >> (12 - sensor_resolution_bits_)) % 4096;
                        }
                    } else {
                        angle_electrical = -1;
                    }
                    break;
            case i_ams[int i].get_ams_position(void)  -> {int position, int direction}:
                    if (sensor_initialized_ == 1){
                        if (enable_aquisition_ == 1){
                            if(settings1_ == 5){
                                abs_pos_ = sensor_resolution_ticks_ - readRotarySensorAngleWithCompensation(sensor_if);
                                measurement_taken_ = 1;
                            }
                            else {
                                abs_pos_ = readRotarySensorAngleWithCompensation(sensor_if);
                                measurement_taken_ = 1;
                            }
                        }

                        if (measurement_taken_ == 1){
                            measurement_taken_ = 0;
                            if((abs_pos_ - abs_pos_previos_) < -sensor_resolution_ticks_/2){
                                pos_multiturn_ += sensor_resolution_ticks_ - abs_pos_previos_ + abs_pos_;
                                direction = 1;
                            }
                            else if ((abs_pos_ - abs_pos_previos_) > sensor_resolution_ticks_/2){
                                pos_multiturn_ += sensor_resolution_ticks_ - abs_pos_ + abs_pos_previos_;
                                direction = -1;
                            }
                            else {
                                pos_multiturn_ += abs_pos_ - abs_pos_previos_;
                            }

                            if ((pos_multiturn_ > max_count_ticks_cw_) || (pos_multiturn_ < -max_count_ticks_ccw_)){
                                pos_multiturn_ = 0;
                            }

                            position = pos_multiturn_;
                            abs_pos_previos_ = abs_pos_;
                        } else {
                            position = 0x7FFFFFFF;
                        }

                    } else {
                        position = 0x7FFFFFFF;
                    }
                    break;
            case i_ams[int i].get_ams_velocity(void) -> int velocity:
                    velocity = velocity_;
                    break;
            case i_ams[int i].get_ams_real_position() -> unsigned int position:
                    if (sensor_initialized_ == 1){
                        position = readRotarySensorAngleWithoutCompensation(sensor_if);
                    } else {
                        position = 0x7FFFFFFF;
                    }
                    break;
            case i_ams[int i].set_ams_config(AMSConfig in_config):
                    enable_aquisition_ = in_config.enable_aquisition;
                    n_pole_pairs_ = in_config.settings2 + 1;
                    sensor_resolution_bits_ = in_config.resolution_bits;
                    settings1_ = in_config.settings1;
                    settings2_ = in_config.settings2;
                    offset_ = in_config.sensor_placement_offset;
                    max_count_ticks_cw_ = in_config.max_count_ticks_cw;
                    max_count_ticks_ccw_ = in_config.max_count_ticks_ccw;

                    int result = initRotarySensor(sensor_if,  settings1_,  settings2_, offset_);
          //          printf("offset_: %i", offset_);

                    if (result > 0){
                        printf("*************************************\n    AMS SENSOR SERVER STARTED\n*************************************\n");
                    } else{
                       printf("*************************************\n    AMS SENSOR SPI ERROR!!!\n*************************************\n");
                       exit(-1);
                    }

                    sensor_resolution_ticks_ = sensor_resolution_ticks_ << sensor_resolution_bits_;
                    velocity_ticks_ = (sensor_resolution_ticks_ >> 8);

                    if(settings1_ == 5){
                        abs_pos_previos_ = sensor_resolution_ticks_ - readRotarySensorAngleWithCompensation(sensor_if);
                    }
                    else{
                        abs_pos_previos_ = readRotarySensorAngleWithCompensation(sensor_if);
                    }
                    sensor_initialized_ = 1;

                    break;

            case i_ams[int i].get_ams_config(void) -> AMSConfig out_config:
                    out_config.settings1 = settings1_;
                    out_config.settings2 = settings2_;
                    out_config.enable_aquisition = enable_aquisition_;
                    out_config.sensor_placement_offset = offset_;
                    out_config.resolution_bits = sensor_resolution_bits_;
                    out_config.max_count_ticks_cw = max_count_ticks_cw_;
                    out_config.max_count_ticks_ccw = max_count_ticks_ccw_;

                    break;
            case i_ams[int i].reset_ams_angle_electrical(unsigned int angle) -> unsigned int out_offset:
                    writeZeroPosition(sensor_if, 0);
                    out_offset = sensor_resolution_ticks_ - ((angle - readRotarySensorAngleWithoutCompensation(sensor_if)) & (sensor_resolution_ticks_-1));
                    writeZeroPosition(sensor_if, out_offset);
                    break;

            case t when timerafter (tmr) :> void:
                    tmr += 250000;
                    if (sensor_initialized_ == 1 && measurement_taken_ == 1){
                        int difference = abs_pos_ - abs_pos_old_vel_est_;
                        if(difference > sensor_resolution_ticks_/2 || difference < -sensor_resolution_ticks_/2){
                            difference = difference_old_;
                        }
                        //ToDo: check the velocity estimate relative to the sensor placement. -1 is temporal there!!!

                        velocity_ = -1 * (difference * 234375) / (velocity_ticks_ * 1000);
               //         velocity_ =  difference;
                            abs_pos_old_vel_est_ = abs_pos_;
                            difference_old_ = difference;

                    }
                    break;
                    //set the calib flag, the server will alway send 0 as electrical angle
            case i_ams[int i].set_ams_calib(int flag):
                    break;
        }
    }

}

void ams_service(sensor_spi_interface &sensor_if, AMSConfig & ams_config, interface AMSInterface server i_ams[5])
{
    //Set freq to 250MHz (always needed for velocity calculation)
//    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

//    if(check_ams_config(ams_config) == ERROR){
//        printstrln("Error while checking the BiSS sensor configuration");
//        return;
//    }

    printstr(">>   SOMANET AMS SENSOR SERVICE STARTING...\n");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << ams_config.resolution_bits);
    int crossover = ticks_per_turn - ticks_per_turn/10;
//    int velocity_loop = (ams_config.velocity_loop * AMS_USEC); //velocity loop time in clock ticks
    int velocity_loop = 1000*250;
//    int velocity_factor = 60000000/ams_config.velocity_loop;
    int velocity_factor = 60000000/1000;
    //position
    unsigned int last_position = 0;
    int last_count = 0;
    int count_offset = 0;
    int turns = 0;
    int calib_flag = 0;
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_count_read = 0;
    unsigned int last_ams_read = 0;


    int result = initRotarySensor(sensor_if,  ams_config.settings1,  ams_config.settings2, ams_config.offset_electrical);
//          printf("offset_: %i", offset_);

    if (result > 0){
        printf("*************************************\n    AMS SENSOR SERVER STARTED\n*************************************\n");
    } else{
       printf("*************************************\n    AMS SENSOR SPI ERROR!!!\n*************************************\n");
       exit(-1);
    }


    //main loop
    while (1) {
        [[ordered]]
        select {
        //send electrical angle for commutation, ajusted with electrical offset
        case i_ams[int i].get_ams_angle_electrical() -> unsigned int angle:
                if (calib_flag == 0) {
//                    t :> time;
//                    if (timeafter(time, last_ams_read + ams_config.timeout)) {
//                        angle = read_ams_sensor_data_fast(ams_ports, ams_before_singleturn_length, ams_config.singleturn_resolution);
//                        t :> last_ams_read;
//                        last_position = angle;
//                    } else
//                        angle = last_position;
//                    if (ams_config.polarity == AMS_POLARITY_INVERTED)
//                        angle = ticks_per_turn - angle;
                    angle = readRotarySensorAngleWithCompensation(sensor_if);
                    if (ams_config.resolution_bits > 12)
                        angle = (ams_config.pole_pairs * (angle >> (ams_config.resolution_bits-12)) ) & 4095;
                    else
                        angle = (ams_config.pole_pairs * (angle << (12-ams_config.resolution_bits)) ) & 4095;
                } else
                    angle = 0;
                break;



        //send count, position and status (error and warning bits), ajusted with count offset and polarity
        case i_ams[int i].get_ams_position() -> { int count, int position }:
//                int count_internal;
//                t :> time;
//                if (timeafter(time, last_count_read + ams_config.timeout)) {
//                    t when timerafter(last_ams_read + ams_config.timeout) :> void;
//                    read_ams_sensor_data(ams_ports, ams_config, data, AMS_FRAME_BYTES);
//                    t :> last_ams_read;
//                    last_count_read = last_ams_read;
//                    { count_internal, position, status } = ams_encoder(data, ams_config);
//                    update_turns(turns, last_count, count_internal, ams_config.multiturn_resolution, ticks_per_turn);
//                    last_count = count_internal;
//                    last_position = position;
//                } else {
//                    count_internal = last_count;
//                    position = last_position;
//                }
//                //add offset
//                if (ams_config.multiturn_resolution) { //multiturn encoder
//                    count = count_internal + count_offset;
//                    if (count < -max_ticks_internal)
//                        count = max_ticks_internal + (count % max_ticks_internal);
//                    else if (count >= max_ticks_internal)
//                        count = (count % max_ticks_internal) - max_ticks_internal;
//                } else //singleturn encoder
//                    count = turns*ticks_per_turn + count_internal  + count_offset;
//                //polarity
//                if (ams_config.polarity == AMS_POLARITY_INVERTED) {
//                    count = -count;
//                    position = ticks_per_turn - position;
//                }
//                //count reset
//                if (count >= ams_config.max_ticks || count < -ams_config.max_ticks) {
//                    count_offset = -count_internal;
//                    count = 0;
//                    status = 0;
//                    turns = 0;
//                }
                count = readRotarySensorAngleWithCompensation(sensor_if);
                position = count;
                break;

        //send count, position and status (error and warning bits) as returned by the encoder (not ajusted)
        case i_ams[int i].get_ams_real_position() -> unsigned int position:
//                t :> time;
//                if (timeafter(time, last_count_read + ams_config.timeout)) {
//                    t when timerafter(last_ams_read + ams_config.timeout) :> void;
//                    read_ams_sensor_data(ams_ports, ams_config, data, AMS_FRAME_BYTES);
//                    t :> last_ams_read;
//                    last_count_read = last_ams_read;
//                    { count, position, status } = ams_encoder(data, ams_config);
//                    update_turns(turns, last_count, count, ams_config.multiturn_resolution, ticks_per_turn);
//                    last_count = count;
//                    last_position = position;
//                } else {
//                    count = last_count;
//                    position = last_position;
//                    status = 0;
//                }
                position = readRotarySensorAngleWithoutCompensation(sensor_if);
                break;

        //send velocity
        case i_ams[int i].get_ams_velocity() -> int out_velocity:
//                if (ams_config.polarity == AMS_POLARITY_NORMAL)
//                    out_velocity = velocity;
//                else
//                    out_velocity = -velocity;
                out_velocity = velocity;
                break;

        //receive new ams_config
        case i_ams[int i].set_ams_config(AMSConfig in_config):
                //update variables which depend on ams_config
//                if (ams_config.clock_dividend != in_config.clock_dividend || ams_config.clock_divisor != in_config.clock_divisor)
//                    configure_clock_rate(ams_ports.clk, in_config.clock_dividend, in_config.clock_divisor) ; // a/b MHz
                if (ams_config.offset_electrical != in_config.offset_electrical)
                    writeZeroPosition(sensor_if, in_config.offset_electrical);
                ams_config = in_config;
//                ams_data_length = ams_config.multiturn_length +  ams_config.singleturn_length + ams_config.status_length;
//                ams_before_singleturn_length = ams_config.multiturn_length + ams_config.singleturn_length - ams_config.singleturn_resolution;
                ticks_per_turn = (1 << ams_config.resolution_bits);
//                crossover = ticks_per_turn - ticks_per_turn/10;
//                max_ticks_internal = (1 << (ams_config.multiturn_resolution -1 + ams_config.singleturn_resolution));
//                velocity_loop = (ams_config.velocity_loop * AMS_USEC);
//                velocity_factor = 60000000/ams_config.velocity_loop;

                break;

        //send ams_config
        case i_ams[int i].get_ams_config() -> AMSConfig out_config:
                out_config = ams_config;
                break;

//        //receive the new count to set and set the offset accordingly
//        case i_ams[int i].reset_ams_position(int new_count):
//                t when timerafter(last_ams_read + ams_config.timeout) :> void;
//                read_ams_sensor_data(ams_ports, ams_config, data, AMS_FRAME_BYTES);
//                t :> last_ams_read;
//                last_count_read = last_ams_read;
//                int count, position;
//                { count, position, void } = ams_encoder(data, ams_config);
//                update_turns(turns, last_count, count, ams_config.multiturn_resolution, ticks_per_turn);
//                last_count = count;
//                last_position = position;
//                if (ams_config.polarity == AMS_POLARITY_INVERTED)
//                    new_count = -new_count;
//                if (ams_config.multiturn_resolution == 0) {
//                    turns = new_count/ticks_per_turn;
//                    count_offset = new_count - ticks_per_turn*turns - count;
//                } else
//                    count_offset = new_count - count;
//                break;

        //receive the new elecrical angle to set and set the offset accordingly
        case i_ams[int i].reset_ams_angle_electrical(unsigned int new_angle) -> unsigned int out_offset:
//                t when timerafter(last_ams_read + ams_config.timeout) :> void;
//                read_ams_sensor_data(ams_ports, ams_config, data, AMS_FRAME_BYTES);
//                t :> last_ams_read;
//                last_count_read = last_ams_read;
//                int count, angle;
//                { count, angle, void } = ams_encoder(data, ams_config);
//                update_turns(turns, last_count, count, ams_config.multiturn_resolution, ticks_per_turn);
//                last_count = count;
//                last_position = angle;
//                if (ams_config.singleturn_resolution > 12)
//                    ams_config.offset_electrical = (new_angle - ams_config.pole_pairs * (angle >> (ams_config.singleturn_resolution-12)) ) & 4095;
//                else
//                    ams_config.offset_electrical = (new_angle - ams_config.pole_pairs * (angle >> (12-ams_config.singleturn_resolution)) ) & 4095;
//                offset = ams_config.offset_electrical;
                writeZeroPosition(sensor_if, 0);
                out_offset = ticks_per_turn - ((new_angle - readRotarySensorAngleWithoutCompensation(sensor_if)) & (ticks_per_turn-1));
                writeZeroPosition(sensor_if, out_offset);
                break;

        //set the calib flag, the server will alway send 0 as electrical angle
        case i_ams[int i].set_ams_calib(int flag):
                calib_flag = flag;
                break;

//        //compute velocity
//        case t when timerafter(next_velocity_read) :> void:
//            next_velocity_read += velocity_loop;
//            int count, position;
//            t :> time;
//            if (timeafter(time, last_count_read + ams_config.timeout)) {
//                t when timerafter(last_ams_read + ams_config.timeout) :> void;
//                read_ams_sensor_data(ams_ports, ams_config, data, AMS_FRAME_BYTES);
//                t :> last_ams_read;
//                last_count_read = last_ams_read;
//                { count, position, void } = ams_encoder(data, ams_config);
//                update_turns(turns, last_count, count, ams_config.multiturn_resolution, ticks_per_turn);
//                last_count = count;
//                last_position = position;
//            } else
//                count = last_count;
//            if (ams_config.multiturn_resolution == 0)
//                count += turns*ticks_per_turn;
//            int difference = count - old_count;
//            if(difference > crossover || difference < -crossover)
//                difference = old_difference;
//            old_count = count;
//            old_difference = difference;
//            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
//            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
//            velocity = (difference * velocity_factor) / ticks_per_turn;
//            break;
        }
    }
}


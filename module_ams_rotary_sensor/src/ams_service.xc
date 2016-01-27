/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <ams_service.h>

#define MILLISECOND 250000 //ticks

static char rotarySensorInitialized = 0;

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initams_ports(AMSPorts &ams_ports){

    if(rotarySensorInitialized != 1){

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
    settings2 |= (config.hysteresis) << 3;
    settings2 |= (config.abi_resolution) << 5;

    return {settings1, settings2};
}

int initRotarySensor(AMSPorts &ams_ports, AMSConfig config){

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

short SPIReadTransaction(AMSPorts &ams_ports, unsigned short reg){

    timer t;
    unsigned ts;
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = addEvenParity(reg);                   //parity

    slave_select(ams_ports.slave_select);                       //start transaction

    spi_master_out_short(ams_ports.spi_interface, reg);          //send command
    ams_ports.spi_interface.mosi <: 0;

    slave_deselect(ams_ports.slave_select);                     //pause
    t:>ts;                                      //for executing
    t when timerafter(ts+50):>void;            //the command
    slave_select(ams_ports.slave_select);                       //on the sensor

    data_in = spi_master_in_short(ams_ports.spi_interface);      //handle response

    slave_deselect(ams_ports.slave_select);                     //end transaction

    return data_in;
}

short SPIWriteTransaction(AMSPorts &ams_ports, unsigned short reg, unsigned short data){

    timer t;
    unsigned ts;
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = addEvenParity(reg);                   //parity

    data &= WRITE_MASK;                         //action
    data = addEvenParity(data);                 //parity

    slave_select(ams_ports.slave_select);                       //start transaction

    spi_master_out_short(ams_ports.spi_interface, reg);          //send command

    slave_deselect(ams_ports.slave_select);                       //pause
    t:>ts;                                                              //for executing
    t when timerafter(ts+AMS_SENSOR_EXECUTING_TIME):>void;             //the command
    slave_select(ams_ports.slave_select);                         //on the sensor

    spi_master_out_short(ams_ports.spi_interface, data);
    ams_ports.spi_interface.mosi <: 0;

    slave_deselect(ams_ports.slave_select);                       //pause
    t:>ts;                                                              //for saving
    t when timerafter(ts+AMS_SENSOR_SAVING_TIME):>void;                //the data
    slave_select(ams_ports.slave_select);                         //on the reg

    data_in = spi_master_in_short(ams_ports.spi_interface);       //handle response
   // printhex(data_in);
   // printstrln("");

    slave_deselect(ams_ports.slave_select);                       //end transaction

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

int check_ams_config(AMSConfig &ams_config){

    if(ams_config.pole_pairs < 1 || ams_config.pole_pairs > 7){
        printstrln("Wrong Hall configuration: wrong pole-pairs");
        return ERROR;
    }

    return SUCCESS;
}


//[[combinable]]
void ams_service(AMSPorts &ams_ports, AMSConfig config, server interface AMSInterface i_AMS[n], unsigned n)
{
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    if(check_ams_config(config) == ERROR){
        printstrln("Error while checking the AMS sensor configuration");
        return;
    }

    printstr(">>   SOMANET AMS SENSOR SERVICE STARTING...\n");

    int n_pole_pairs = config.pole_pairs;
    int segm_resolution = config.sensor_resolution/n_pole_pairs;
    float norm_factor = segm_resolution/4096.0;
    int angle_electrical_rotation = 0;
    int segm_num = 0;
    int _abs_pos = 0, _abs_pos_previos = 0;
    int direction = 0;

    int vel_previous_position = 0, vel_old_difference = 0;
    int difference_velocity;
    int velocity = 0;
    timer t_velocity;
    unsigned int ts_velocity;

    timer t_read_sensor;
    unsigned int ts_read_sensor;

    initRotarySensor(ams_ports, config);

    _abs_pos_previos = readRotarySensorAngleWithCompensation(ams_ports);//readRotarySensorAngleWithoutCompensation(ams_ports);

    long long pos_multiturn = _abs_pos_previos;

    t_velocity :> ts_velocity;
    t_read_sensor :> ts_read_sensor;

    while(1)
    {
#pragma ordered
        select {
            case t_read_sensor when timerafter(ts_read_sensor + 12*2500) :> ts_read_sensor:
                    if(config.direction == AMS_DIR_CCW){
                        _abs_pos = config.sensor_resolution - readRotarySensorAngleWithCompensation(ams_ports);//readRotarySensorAngleWithoutCompensation(ams_ports);
                    }
                    else {
                        _abs_pos = readRotarySensorAngleWithCompensation(ams_ports);//readRotarySensorAngleWithoutCompensation(ams_ports);
                    }

                    if((_abs_pos - _abs_pos_previos) < -config.sensor_resolution/2) {
                        pos_multiturn += config.sensor_resolution - _abs_pos_previos + _abs_pos;
                        direction = 1;
                    }
                    else if ((_abs_pos - _abs_pos_previos) > config.sensor_resolution/2) {
                        pos_multiturn += config.sensor_resolution - _abs_pos + _abs_pos_previos;
                        direction = -1;
                    }
                    else {
                        pos_multiturn += _abs_pos - _abs_pos_previos;
                    }
                    _abs_pos_previos = _abs_pos;
                    break;

            case i_AMS[int i].get_ams_angle(void) -> int angle:
                    segm_num = (int)_abs_pos/segm_resolution;
                    angle_electrical_rotation = (_abs_pos - segm_resolution * segm_num)/norm_factor;
                    angle = angle_electrical_rotation;
                    break;

            case i_AMS[int i].get_ams_position(void) -> int out_position:
                   out_position = _abs_pos;
                   break;

            case i_AMS[int i].get_ams_velocity(void) -> int out_velocity:
                    out_velocity = velocity;
                    break;

            case i_AMS[int i].get_ams_direction(void) -> int out_direction:
                    out_direction = direction;
                    break;

            case i_AMS[int i].get_ams_position_absolute(void) -> int out_position:
                    out_position = pos_multiturn;
                    break;

            case i_AMS[int i].reset_ams_absolute_position(int offset):
                    pos_multiturn = offset;
                    break;

            case i_AMS[int i].get_ams_config(void) -> AMSConfig out_config:
                    out_config = config;
                    break;

            case i_AMS[int i].set_ams_config(AMSConfig in_config):
                    config = in_config;
                    break;

            case t_velocity when timerafter(ts_velocity + MILLISECOND) :> ts_velocity:

                  difference_velocity = pos_multiturn - vel_previous_position;
                  //if ( (difference_velocity > qei_crossover_velocity) || (difference_velocity < -qei_crossover_velocity) )
                  //    difference_velocity = vel_old_difference;

                  vel_previous_position = pos_multiturn;
                  vel_old_difference = difference_velocity;

                  velocity = (difference_velocity*60000)/(config.sensor_resolution);

                  break;
        }
    }
}

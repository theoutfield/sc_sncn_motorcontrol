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
#include <ctype.h>

extern char start_message[];
//in port INTPIN = on tile[IFM_TILE]: XS1_PORT_1A;
//int check_pin;

//FixMe extend config structure
//#define CONFIG_EXTENDED

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void initspiPorts(SPIPorts &spi_ports)
{
    spi_master_init(spi_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
    slave_deselect(spi_ports.slave_select); // Ensure slave select is in correct start state
}

/**
 * @return 1 if uneven parity, 0 even parity
 */
uint8_t calcParity(unsigned short bitStream)
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
unsigned short add_EvenParity(unsigned short bitStream){
     return (bitStream |= (calcParity(bitStream) << 0));      //set parity bit to this
}

/*
 * Check if the parity bit [0] is right.
 * Returns 1 if true, 0 if false.
 *
 */
unsigned char check_EvenParity(unsigned short bitStream){
     return (calcParity(bitStream) == ((bitStream >> 0) & 1));    //comparison the parity bit the the real one
}

short ReadTransactionSPI(SPIPorts &spi_ports, unsigned short reg) {
    unsigned short data_in = 0;

    reg |= READ_MASK;                           //read command
    reg = reg <<1;
    reg = add_EvenParity(reg);                   //parity

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

short WriteTransactionSPI(SPIPorts &spi_ports, unsigned short reg, unsigned short data) {
    unsigned short data_in = 0;

    reg &= WRITE_MASK;                          //action
    reg = reg << 1;
    reg = add_EvenParity(reg);                   //parity

  //  data &= WRITE_MASK;                         //action
    data = data << 1;
    data = add_EvenParity(data);                 //parity

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

    data_in = ReadTransactionSPI(spi_ports, ADDR_ErrorStatus);

    if(check_EvenParity(data_in)){            //check right parity

        return ((data_in & BITS_1_MASK) >> 6);         //get WOW bit

    }else{

        return PARITY_ERROR;
    }
}

int readAngleValue(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    delay_microseconds(20);

    if (checkWOWBit(spi_ports) == 0x0)
    {

    data_in = ReadTransactionSPI(spi_ports, ADDR_AngularData);
   // printstrln("sensor ready! read now..");
    }
    else
    {
        return SENSOR_NOT_READY;
    }
    if(check_EvenParity(data_in)){                     //check right parity

        return ((data_in & BITS_10_MASK ) >> 2);        //remove unused bits
      //  return (data_in & BITS_10_MASK);            //remove unused bits

    }else{

       // printstrln("parity error");
        return PARITY_ERROR;

    }
}

int readAutomaticGainControl(SPIPorts &spi_ports){

    unsigned short data_in = 0;

    data_in = ReadTransactionSPI(spi_ports, ADDR_AGC);

    if(check_EvenParity(data_in)){                     //check right parity

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
    data_in = WriteTransactionSPI(spi_ports, ADDR_PorOFF, data);

    if(check_EvenParity(data_in)){            //check right parity

        if((data_in & BITS_8_MASK) == data){

            return SUCCESS_WRITING;
        }else{

            return ERROR_WRITING;
        }
    }else{

        return PARITY_ERROR;
    }
}
int initas5050a(SPIPorts &spi_ports){

    initspiPorts(spi_ports);

    delay_microseconds(600);
    return 0;
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

void as5050a_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3])
{

    if (as5050a_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    position_feedback_config.offset &= (position_feedback_config.resolution-1);
//    if (initRotarySensor(spi_ports,  position_feedback_config) != SUCCESS_WRITING) {
//        printstrln("Error with SPI REM_14 sensor");
//        position_feedback_config.sensor_type = 0;
//        return;
//    }

    initas5050a(spi_ports);

    printstr(start_message);
    printstrln("AS5050A");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;

    int velocity_loop = 0;
#ifdef CONFIG_EXTENDED
    int velocity_loop = position_feedback_config.as5050a_config.velocity_loop * as5050a_USEC; //velocity loop time in clock ticks

    int velocity_factor = 60000000/position_feedback_config.as5050a_Config.velocity_loop;
#endif
    //position
    unsigned int last_position = 0;
    int count = 0;
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_read = 0;
    unsigned int last_velocity_read= 0;

    //FixMe find out if the notification is still required
    int notification = 0;//MOTCTRL_NTF_EMPTY;

    int actual_velocity = 0;
    int actual_count = 0;
    unsigned int actual_position = 0;
    unsigned int actual_angle = 0;
    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;

    //first read
    last_position = readAngleValue(spi_ports);
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
                   /* check for INT/ is low to continue */
//                   INTPIN :> check_pin;
//                    if (check_pin == 0) {
                    angle = readAngleValue(spi_ports);
                    t :> last_read;
                    multiturn(count, last_position, angle, position_feedback_config.resolution);
                    last_position = angle;
               // } else
              //  angle = last_position;
               // angle = (position_feedback_config.pole_pairs * (angle << 2) ) & 4095;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                t :> time;
                //INTPIN :> check_pin;
                //if (check_pin == 0) {
                    position = readAngleValue(spi_ports);
                    t :> last_read;
                    multiturn(count, last_position, position, position_feedback_config.resolution);
                    last_position = position;
                //} else
                  //  position = last_position;
                //count reset
#ifdef CONFIG_EXTENDED
                if (count >= position_feedback_config.as5050a_config.max_ticks || count < -position_feedback_config.as5050a_config.max_ticks)
                    count = 0;
#endif
                out_count = count;
                break;

        //send position
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                position = readAngleValue(spi_ports);
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
                    initas5050a(spi_ports);
                else if (position_feedback_config.offset != in_config.offset)
      writeZeroPosition(spi_ports, in_config.offset);
                position_feedback_config = in_config;
                crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
#ifdef CONFIG_EXTENDED
                velocity_loop = position_feedback_config.as5050a_config.velocity_loop * as5050a_USEC;
                velocity_factor = 60000000/position_feedback_config.as5050a_config.velocity_loop;   /* velocity_loop for as5050a ?*/
#endif
                //FixMe find out if the notification is still required
                notification = 1;//MOTCTRL_NTF_CONFIG_CHANGED;
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
                last_position = readAngleValue(spi_ports);
                t :> last_read;
                count = new_count;
                break;

        //receive the new elecrical angle to set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                //writeZeroPosition(spi_ports, 0);
                int position = readAngleValue(spi_ports)- position_feedback_config.offset;
                out_offset = (position_feedback_config.resolution - ((new_angle >> 2) / position_feedback_config.pole_pairs) + position) & (position_feedback_config.resolution-1);
                //out_offset = (position_feedback_config.resolution - ((new_angle << 2) / position_feedback_config.pole_pairs) + position) & (position_feedback_config.resolution-1);
                //          writeZeroPosition(spi_ports, out_offset);
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
            position = readAngleValue(spi_ports);
            t :> last_read;
            multiturn(count, last_position, position, position_feedback_config.resolution);
            last_position = position;

            angle = (position_feedback_config.pole_pairs * (position << 2) ) & 4095;

            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
//            velocity = (difference * velocity_factor) / ticks_per_turn;
            velocity = (difference * (60000000/((int)(last_read-last_velocity_read)/as5050a_USEC))) / position_feedback_config.resolution;
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
                next_velocity_read = end_time + as5050a_USEC;
            break;
        }
    }
}

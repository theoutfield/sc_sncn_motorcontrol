/**
 * @file qei_server.xc
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/


#include <qei_service.h>
#include <stdlib.h>
#include <xs1.h>
#include <refclk.h>
#include <xscope.h>
#include "print.h"
#include <internal_config.h>

#define MILLISECOND 250000 //ticks
//#pragma xta command "analyze loop qei_loop"
//#pragma xta command "set required - 1.0 us"
// Order is 00 -> 10 -> 11 -> 01
// Bit 3 = Index
static const unsigned char lookup[16][4] = {
    { 5, 4, 6, 5 }, // 00 00
    { 6, 5, 5, 4 }, // 00 01
    { 4, 5, 5, 6 }, // 00 10
    { 5, 6, 4, 5 }, // 00 11
    { 0, 0, 0, 0 }, // 01 xx
    { 0, 0, 0, 0 }, // 01 xx
    { 0, 0, 0, 0 }, // 01 xx
    { 0, 0, 0, 0 }, // 01 xx

    { 5, 4, 6, 5 }, // 10 00
    { 6, 5, 5, 4 }, // 10 01
    { 4, 5, 5, 6 }, // 10 10
    { 5, 6, 4, 5 }, // 10 11
    { 0, 0, 0, 0 }, // 11 xx
    { 0, 0, 0, 0 }, // 11 xx
    { 0, 0, 0, 0 }, // 11 xx
    { 0, 0, 0, 0 }  // 11 xx
};

int check_qei_config(QEIConfig &qei_config)
{

    if(qei_config.index < 3  || qei_config.index > 4){
        printstrln("Wrong QEI configuration: wrong type");
        return ERROR;
    }

    if(qei_config.sensor_polarity < 0  || qei_config.sensor_polarity > 1){
        printstrln("Wrong QEI configuration: wrong polarity");
        return ERROR;
    }

    if(qei_config.poles < 0  || qei_config.poles > 15){
        printstrln("Wrong QEI configuration: wrong pole-pairs");
        return ERROR;
    }

    if(qei_config.real_counts < 0){
        printstrln("Wrong QEI configuration: wrong resolution");
        return ERROR;
    }

    return SUCCESS;
}


#pragma unsafe arrays
void qei_service(QEIPorts & encoder_ports, QEIConfig qei_config,
                    interface QEIInterface server i_qei[5])
{
    //Set freq to 250MHz (always needed for velocity calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

               // to compute velocity from qei
    if(check_qei_config(qei_config) == ERROR){
        printstrln("Error while checking the QEI sensor configuration");
        return;
    }

    printstr("*************************************\n    QEI SENSOR SERVER STARTING\n*************************************\n");

    int max_ticks_per_turn = qei_config.real_counts;
    int position = 0;
    unsigned int v;


    unsigned int ok = 0;
    unsigned int old_pins = 0;
    unsigned int new_pins;

    int previous_position = 0;
    int count = 0;
    int first = 1;
    int max_count_actual = 2147483648; //TODO remove variable and make a define
    int difference = 0;
    int direction = 0;
    int qei_max = max_ticks_per_turn;
    int qei_type = qei_config.index;            // TODO use to disable sync for no-index
    int init_state = INIT;

    int qei_crossover = (qei_max*19)/100;
    int qei_count_per_hall = qei_config.real_counts / qei_config.poles;
    int offset_fw = 0;
    int offset_bw = 0;
    int calib_fw_flag = 0;
    int calib_bw_flag = 0;
    int sync_out = 0;
    int status = 0;
    int flag_index = 0;
    unsigned int new_pins_1;

    int qei_crossover_velocity = qei_config.real_counts - qei_config.real_counts/10;
    int vel_previous_position = 0, vel_old_difference = 0;
    int difference_velocity;
    int velocity = 0;
    timer t_velocity;
    unsigned int ts_velocity;

    t_velocity :> ts_velocity;

    encoder_ports.p_qei :> new_pins;

    while (1) {
#pragma xta endpoint "qei_loop"
#pragma ordered
        select {
        case encoder_ports.p_qei when pinsneq(new_pins) :> new_pins :
            encoder_ports.p_qei :> new_pins_1;
            encoder_ports.p_qei :> new_pins_1;
            if(new_pins_1 == new_pins) {
                encoder_ports.p_qei :> new_pins;
                if(new_pins_1 == new_pins) {
                    v = lookup[new_pins][old_pins];

                    if(qei_type == QEI_WITH_NO_INDEX) {
                        { v, position } = lmul(1, position, v, -5);
                        if(position >= qei_config.real_counts )
                            position = 0;
                        else if(position <= -qei_config.real_counts )
                            position = 0;
                    } else {
                        if (!v) {
                            flag_index = 1;
                            ok = 1;
                            position = 0;
                        } else {
                            { v, position } = lmul(1, position, v, -5);
                            flag_index = 0;
                        }
                    }


                    //	xscope_int(0, position);

                    old_pins = new_pins & 0x3;

                    if (first == 1) {
                        previous_position = position;
                        first = 0;
                    }

                    if (previous_position != position) {
                        difference = position - previous_position;
                        //xscope_int(1, difference);
                        if (difference >= qei_crossover) {
                            if (qei_config.sensor_polarity == QEI_POLARITY_NORMAL) {
                                count = count - 1;
                            } else {
                                count = count + 1;
                            }
                            sync_out = offset_fw;  //valid needed
                            calib_fw_flag = 1;
                            direction = -1;
                        }
                        else if (difference <= -qei_crossover) {
                            if (qei_config.sensor_polarity == QEI_POLARITY_NORMAL) {
                                count = count + 1;
                            } else {
                                count = count - 1;
                            }
                            sync_out = offset_bw;
                            calib_bw_flag = 1;
                            direction = +1;
                        }
                        else if (difference <= 2 && difference > 0) {
                            if (qei_config.sensor_polarity == QEI_POLARITY_NORMAL) {
                                count = count + difference;
                                sync_out = sync_out + difference;
                            } else {
                                count = count - difference;
                                sync_out = sync_out - difference;
                            }
                            direction = -1;
                        }
                        else if (difference < 0 && difference >= -2) {
                            if (qei_config.sensor_polarity == QEI_POLARITY_NORMAL) {
                                count = count + difference;
                                sync_out = sync_out + difference;
                            } else {
                                count = count - difference;
                                sync_out = sync_out - difference;
                            }
                            direction = 1;
                        }
                        previous_position = position;
                    }

                    if (sync_out < 0) {
                        sync_out = qei_count_per_hall + sync_out;
                    }

                    if(count >= max_count_actual || count <= -max_count_actual) {
                        count=0;
                    }

                    if(sync_out >= qei_count_per_hall ) {
                        sync_out = 0;
                    }
                }
            }
            //xscope_int(0, position);
            //xscope_int(1, count);

            break;

        case i_qei[int i].get_qei_position() -> {unsigned int out_count, unsigned int out_valid}:

                out_count = count;
                out_count &= (max_ticks_per_turn - 1);
                out_valid = ok;
                break;

        case i_qei[int i].get_qei_position_absolute() -> {int out_count, int out_direction}:

                out_count = count;
                out_direction = direction;
                break;

        case i_qei[int i].get_qei_sync_position() -> {int out_position, int out_calib_fw, int out_calib_bw}:

                out_position = sync_out;
                out_calib_fw = calib_fw_flag;
                out_calib_bw = calib_bw_flag;
                break;

        case i_qei[int i].get_qei_velocity() -> int out_velocity:

                out_velocity = velocity;
                 break;

        case i_qei[int i].set_qei_sync_offset(int in_fw, int in_bw):

                offset_fw = in_fw;
                offset_bw = in_bw;
                calib_bw_flag = 0;
                calib_fw_flag = 0;
                 break;

        case i_qei[int i].reset_qei_count(int in_offset):

                 count = in_offset;
                 break;
        case i_qei[int i].getQEIConfig() -> QEIConfig out_config:

                out_config = qei_config;
                break;

        case i_qei[int i].setQEIConfig(QEIConfig in_config):

                qei_config = in_config;
                break;

        case i_qei[int i].checkBusy() -> int out_status:

                out_status = init_state;
                break;

        case t_velocity when timerafter(ts_velocity + MILLISECOND) :> ts_velocity:

              difference_velocity = count - vel_previous_position;
              if(difference_velocity > qei_crossover_velocity)
                  difference_velocity = vel_old_difference;
              else if(difference_velocity < -qei_crossover_velocity)
                  difference_velocity = vel_old_difference;

              vel_previous_position = count;
              vel_old_difference = difference_velocity;

              velocity = (difference_velocity*60000)/(qei_config.real_counts);

              break;

        }

        if (status == 1) {
            status = 0;
            qei_max = max_ticks_per_turn;
            qei_type = qei_config.index;
            qei_crossover = (qei_max*19)/100;
            qei_count_per_hall = qei_config.real_counts / qei_config.poles;
        }
#pragma xta endpoint "qei_loop_end_point"
    }
}

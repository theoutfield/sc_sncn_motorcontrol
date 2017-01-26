/**
 * @file qei_server.xc
 * @brief Incremental Encoder Service Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <qei_service.h>
#include <limits.h>
#include "print.h"
#include <mc_internal_constants.h>

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

extern char start_message[];

int check_qei_config(PositionFeedbackConfig &position_feedback_config)
{
    if (position_feedback_config.qei_config.index_type < 3  || position_feedback_config.qei_config.index_type > 4) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong type");
        return ERROR;
    }

    if (position_feedback_config.polarity != -1 && position_feedback_config.polarity != 1) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong polarity");
        return ERROR;
    }

    if (position_feedback_config.resolution < 0) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong resolution");
        return ERROR;
    }

    return SUCCESS;
}

#pragma unsafe arrays
void qei_service(QEIHallPort &qei_hall_port, port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
                 client interface shared_memory_interface ?i_shared_memory,
                 server interface PositionFeedbackInterface i_position_feedback[3])
{


    if (QEI_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }


#ifdef DEBUG_POSITION_FEEDBACK
    if (check_qei_config(position_feedback_config) == ERROR) {
        position_feedback_config.sensor_type = 0;
        return;
    }

    printstr(start_message);
    printstrln("QEI");
#endif

    //position_feedback_config.qei_config.max_ticks_per_turn = position_feedback_config.qei_config.real_counts;
    int position = 0;
    unsigned int v;

    unsigned int ok = 0;
    unsigned int old_pins = 0;
    unsigned int new_pins;

    int previous_position = 0;
    int count = 0;
    int first = 1;
    int const config_max_ticks = INT_MAX;//position_feedback_config.qei_config.max_ticks;
    int const config_min_ticks = INT_MIN;
    int difference = 0;
    int direction = 0;
    int qei_type = position_feedback_config.qei_config.index_type;            // TODO use to disable sync for no-index
//    int init_state = INIT;

    int qei_crossover = (position_feedback_config.resolution * 19) / 100;
    int qei_count_per_hall = position_feedback_config.resolution;   // / position_feedback_config.qei_config.poles;
    int offset_fw = 0;
    int offset_bw = 0;
    int calib_fw_flag = 0;
    int calib_bw_flag = 0;
    int sync_out = 0;
    int flag_index = 0;
    unsigned int new_pins_1;

    int qei_crossover_velocity = position_feedback_config.resolution - position_feedback_config.resolution / 10;
    int vel_previous_position = 0, vel_old_difference = 0;
    int difference_velocity;
    int velocity = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    timer t_velocity;
    unsigned int ts_velocity;

    t_velocity :> ts_velocity;

    qei_hall_port.p_qei_hall :> new_pins;

    int loop_flag = 1;
    while (loop_flag) {
#pragma xta endpoint "qei_loop"
#pragma ordered
        select {
            case qei_hall_port.p_qei_hall when pinsneq(new_pins) :> new_pins :
                qei_hall_port.p_qei_hall :> new_pins_1;
                qei_hall_port.p_qei_hall :> new_pins_1;
                if (new_pins_1 == new_pins) {
                    qei_hall_port.p_qei_hall :> new_pins;
                    if (new_pins_1 == new_pins) {
                        v = lookup[new_pins][old_pins];

                        if (qei_type == QEI_WITH_NO_INDEX) {
                            { v, position } = lmul(1, position, v, -5);
                            if (position >= position_feedback_config.resolution) {
                                position = 0;
                            } else if (position <= -position_feedback_config.resolution) {
                                position = 0;
                            }
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

                        old_pins = new_pins & 0x3;

                        if (first == 1) {
                            previous_position = position;
                            first = 0;
                        }

                        if (previous_position != position) {
                            difference = position - previous_position;
                            //xscope_int(1, difference);
                            if (difference >= qei_crossover) {
                                if (position_feedback_config.polarity == QEI_POLARITY_NORMAL) {
                                    count = count - 1;
                                } else {
                                    count = count + 1;
                                }
                                sync_out = offset_fw;  //valid needed
                                calib_fw_flag = 1;
                                direction = -1;
                            } else if (difference <= -qei_crossover) {
                                if (position_feedback_config.polarity == QEI_POLARITY_NORMAL) {
                                    count = count + 1;
                                } else {
                                    count = count - 1;
                                }
                                sync_out = offset_bw;
                                calib_bw_flag = 1;
                                direction = +1;
                            } else if (difference <= 2 && difference > 0) {
                                if (position_feedback_config.polarity == QEI_POLARITY_NORMAL) {
                                    count = count + difference;
                                    sync_out = sync_out + difference;
                                } else {
                                    count = count - difference;
                                    sync_out = sync_out - difference;
                                }
                                direction = -1;
                            } else if (difference < 0 && difference >= -2) {
                                if (position_feedback_config.polarity == QEI_POLARITY_NORMAL) {
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

                        if (count >= config_max_ticks || count <= config_min_ticks) {
                            count=0;
                        }

                        if (sync_out >= qei_count_per_hall ) {
                            sync_out = 0;
                        }
                    }

                    if (!isnull(i_shared_memory)) {
                        if (position_feedback_config.enable_push_service == PushPosition || position_feedback_config.enable_push_service == PushAll) {
                            i_shared_memory.write_velocity_position(velocity, count);
                        }
                    }
                }

                break;

            case i_position_feedback[int i].get_notification() -> int out_notification:

                out_notification = notification;
                break;

//            case i_position_feedback[int i].get_qei_position() -> {unsigned int out_count, unsigned int out_valid}:
//
//                out_count = count;
//                out_count &= (config_qei_changes_per_turn - 1);
//                out_valid = ok;
//                break;

            case i_position_feedback[int i].get_position() -> { int out_count, unsigned int out_position, unsigned int status }:

                out_count = count;
                out_position = count & (position_feedback_config.resolution - 1);
                break;

//            case i_position_feedback[int i].get_qei_sync_position() -> {int out_position, int out_calib_fw, int out_calib_bw}:
//
//                out_position = sync_out;
//                out_calib_fw = calib_fw_flag;
//                out_calib_bw = calib_bw_flag;
//                break;

            case i_position_feedback[int i].get_velocity() -> int out_velocity:

                out_velocity = velocity;
                break;

//            case i_position_feedback[int i].set_qei_sync_offset(int in_fw, int in_bw):
//
//                offset_fw = in_fw;
//                offset_bw = in_bw;
//                calib_bw_flag = 0;
//                calib_fw_flag = 0;
//                break;

            case i_position_feedback[int i].set_position(int in_count):

                 count = in_count;
                 break;

            case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:

                out_config = position_feedback_config;
                break;

            case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):

                position_feedback_config = in_config;
                qei_crossover_velocity = position_feedback_config.resolution - position_feedback_config.resolution / 10;
                // max_count_actual = position_feedback_config.qei_config.max_ticks;
                qei_type = position_feedback_config.qei_config.index_type;
                qei_crossover = (position_feedback_config.resolution * 19) / 100;
                qei_count_per_hall = position_feedback_config.resolution;// / position_feedback_config.qei_config.poles;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }
                break;

//            case i_position_feedback[int i].check_busy() -> int out_status:
//
//                out_status = init_state;
//                break;

//            case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
//                out_ticks_per_turn = position_feedback_config.resolution;
//                break;

            case i_position_feedback[int i].get_angle() -> unsigned int out_angle:
                break;

//            case i_position_feedback[int i].set_angle(unsigned int in_angle) -> unsigned int out_offset:
//                break;

//            case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int out_position,  unsigned int out_status}:
//                break;

            case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int out_status:
                break;

            case i_position_feedback[int i].exit():
                loop_flag = 0;
                continue;

            //gpio read
            case i_position_feedback[int i].gpio_read(int gpio_number) -> int out_value:
                    out_value = gpio_read(gpio_ports, position_feedback_config, gpio_number);
                    break;

            //gpio_write
            case i_position_feedback[int i].gpio_write(int gpio_number, int in_value):
                    gpio_write(gpio_ports, position_feedback_config, gpio_number, in_value);
                    break;

            case t_velocity when timerafter(ts_velocity + (1000*QEI_USEC)) :> ts_velocity:

                difference_velocity = count - vel_previous_position;
                if (difference_velocity > qei_crossover_velocity) {
                    difference_velocity = vel_old_difference;
                } else if(difference_velocity < -qei_crossover_velocity) {
                    difference_velocity = vel_old_difference;
                }

                vel_previous_position = count;
                vel_old_difference = difference_velocity;

                velocity = (difference_velocity * 60000) / position_feedback_config.resolution;

                //gpio
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory);

                break;

        }
#pragma xta endpoint "qei_loop_end_point"
    }
}

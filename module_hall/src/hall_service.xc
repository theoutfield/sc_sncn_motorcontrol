/**
 * @file hall_server.xc
 * @brief Hall Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */
#include <xs1.h>
#include <hall_service.h>
#include <limits.h>
#include <filter_blocks.h>
#include <refclk.h>
#include <stdlib.h>
#include <print.h>
#include <xscope.h>

#include <mc_internal_constants.h>

int check_hall_config(HallConfig &hall_config){

    if (hall_config.pole_pairs < 1 || hall_config.pole_pairs > 11) {
        printstrln("hall_service: ERROR: Wrong Hall configuration: wrong pole-pairs");
        return ERROR;
    }

    return SUCCESS;
}

void sector_transition(hall_variables& hv, int pin_state);
void hall_calculate_speed(hall_variables& hv);
void hall_calculate_angle(hall_variables& hv);
void speed_LPF(hall_variables& hv);

static inline void multiturn(int &count, int last_position, int position, int ticks_per_turn) {
        int difference = position - last_position;
        if (difference >= ticks_per_turn/2)
            count = count + difference - ticks_per_turn;
        else if (-difference >= ticks_per_turn/2)
            count = count + difference + ticks_per_turn;
        else
            count += difference;
}

//[[combinable]]
void hall_service(HallPorts &hall_ports, PositionFeedbackConfig &position_feedback_config,
        client interface shared_memory_interface ?i_shared_memory,
                server interface PositionFeedbackInterface i_position_feedback[3])
{
//    //Set freq to 250MHz (always needed for velocity calculation)
//    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    if (check_hall_config(position_feedback_config.hall_config) == ERROR) {
        printstrln("hall_service: ERROR: Error while checking the Hall sensor configuration");
        position_feedback_config.sensor_type = 0;
        return;
    }

    printstr(">>   SOMANET HALL SENSOR SERVICE STARTING...\n");

    //    timer tx, tmr;
    //    unsigned int ts;
    //
    //    unsigned int angle1 = 0;            // newest angle (base angle on hall state transition)
    //    unsigned int delta_angle = 0;
    //    unsigned int angle = 0;
    //
    //    unsigned int iCountMicroSeconds = 0;
    //    unsigned int iPeriodMicroSeconds = 0;
    //    unsigned int iTimeCountOneTransition = 0;
    //    unsigned int iTimeSaveOneTransition = 0;
    //
    //    unsigned int pin_state = 0;         // newest hall state
    //    unsigned int pin_state_last = 0;
    //    unsigned int pin_state_monitor = 0;
    //    unsigned int new1 = 0;
    //    unsigned int new2 = 0;
    //    unsigned int uHallNext = 0;
    //    unsigned int uHallPrevious = 0;
    //    int xreadings = 0;
    //
    //    int iHallError = 0;
    //    int direction = 0;
    //
    //    int position = 0;
    //    int previous_position = 0;
    //    int count = 0;
    //    int first = 1;
    //    int time_elapsed = 0;
    //    int init_state = INIT;
    //
    //    timer t1;
    //    int time1;
    //    int init_velocity = 0;
    //
    //    int previous_position1 = 0;
    //    int velocity = 0;
    //    int difference1 = 0;
    //    int old_difference = 0;
    //    int filter_length = FILTER_LENGTH_HALL;
    //    int filter_buffer[FILTER_LENGTH_HALL] = {0};
    //    int index = 0;
    //    int raw_velocity = 0;
    //    int hall_crossover = (4096 * 9 )/10;
    //    int status = 0; //1 changed
    //
    //    int config_max_ticks_per_turn = position_feedback_config.hall_config.pole_pairs * HALL_TICKS_PER_ELECTRICAL_ROTATION;
    //    int const config_hall_max_count = INT_MAX;
    //    int const config_hall_min_count = INT_MIN;
    //
    //    int notification = MOTCTRL_NTF_EMPTY;
    //
    //    /* Init hall sensor */
    //    hall_ports.p_hall :> pin_state;
    //    pin_state &= 0x07;
    //    pin_state_monitor = pin_state;
    //    switch (pin_state) {
    //        case 3:
    //            angle = 0;
    //            break;
    //        case 2:
    //            angle = 682;
    //            break; //  60
    //        case 6:
    //            angle = 1365;
    //            break;
    //        case 4:
    //            angle = 2048;
    //            break; // 180
    //        case 5:
    //            angle = 2730;
    //            break;
    //        case 1:
    //            angle = 3413;
    //            break; // 300 degree
    //    }
    //
    //    t1 :> time1;
    //    tmr :> ts;
    //
    //    int loop_flag = 1;





    ///////////////////////////////////////*********************************************

    timer tx;
    unsigned int time1=0;

    unsigned int hall_new_change_moment;
    unsigned int hall_previous_change_moment;

    unsigned int hall_waiting_time;
    unsigned int hall_transition_period_at_low_speeds;
    unsigned int speed_reduction_factor;

    int hall_direction;
    hall_direction = 0;

    int hall_sector_and_state;
    hall_sector_and_state=0;

    int angle_out=0, last_angle=0, speed_out=0, count = 0;
    int init_angle = 0;

    int hall_sector_and_state_temp;

    // This variable must be an unsigned integer. other formats will lead to error
    unsigned int hall_sector_number_and_state_temp;
    unsigned int hall_transient_period_temp;

    int hall_position_temp;

    unsigned int hall_last_state_period;
    hall_last_state_period=HALL_TRANSITION_PERIOD_MAX;

    unsigned int hall_state_old;
    hall_state_old = 0;

    unsigned int hall_state_new;
    hall_state_new = 0;

    unsigned int hall_state_1  ;
    unsigned int hall_state_2  ;

    unsigned int hall_state_next;
    unsigned int hall_state_previous;

    unsigned int hall_transition_timeout;
    hall_transition_timeout=HALL_PERIOD_MAX;

    unsigned int hall_last_period;

    unsigned int hall_period0;
    unsigned int hall_period1;
    unsigned int hall_period2;
    unsigned int hall_period3;
    unsigned int hall_period4;
    unsigned int hall_period5;

    int hall_error;
    hall_error=0;

    // it is very important to set this variable to 0 at the beginning of the code
    unsigned int hall_stable_states  ;
    hall_stable_states = 0;

    int hall_transition_time;
    hall_transition_time=0;

    hall_variables hv;


    // filter initialization:
    hv.hall_filter_order = 3;

    hv.h[0] = 300;
    hv.h[1] = 380;
    hv.h[2] = 320;

    // emptying the buffer of filter
    hv.hall_filter_buffer[0] = 0;
    hv.hall_filter_buffer[1] = 0;
    hv.hall_filter_buffer[2] = 0;

    hv.hall_filter_index_newest=0;


    // clock frequency of defined timers in hall section
    hv.hall_f_clock = 250000000; //250 MHz of clock frequency
    // motor pole pairs
    hv.hall_pole_pairs = position_feedback_config.hall_config.pole_pairs;
    hv.hall_transition_period_at_1rpm = (hv.hall_f_clock / (hv.hall_pole_pairs*6)) * 60 ;

    hv.sensor_polarity=position_feedback_config.hall_config.polarity;


    do
    {
        hall_ports.p_hall :> hall_state_1;
        hall_ports.p_hall :> hall_state_2;
    }while(hall_state_1 != hall_state_2);

    hall_state_new = hall_state_1;
    hall_state_old = hall_state_1;



    ///////////////////////////////////////*********************************************














    tx :> time1;


    while (1) {
        //#pragma xta endpoint "hall_loop"
        //[[ordered]] //FixMe ordered is not supported for combinable functions
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:

                //                out_notification = notification;
                break;

        case i_position_feedback[int i].get_angle() -> unsigned int out_angle:
                //                out_angle = angle;
                break;

        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int out_position }:
                //                out_count = count;
                //                out_position = angle;
                break;

        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                //                out_velocity = raw_velocity;
                break;

        case i_position_feedback[int i].set_position(int in_count):
                //                count = in_count;
                break;

        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                //                out_config = position_feedback_config;
                break;

        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                //
                //                position_feedback_config = in_config;
                //                config_max_ticks_per_turn = position_feedback_config.hall_config.pole_pairs * HALL_TICKS_PER_ELECTRICAL_ROTATION;
                //                status = 1;
                //
                //                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                //                // TODO: Use a constant for the number of interfaces
                //                for (int i = 0; i < 3; i++) {
                //                    i_position_feedback[i].notification();
                //                }
                break;

        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                //                out_ticks_per_turn = HALL_TICKS_PER_ELECTRICAL_ROTATION;
                break;

        case i_position_feedback[int i].set_angle(unsigned int in_angle) -> unsigned int out_offset:
                break;

        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int out_position,  unsigned int out_status}:
                break;

        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int out_status:
                break;

        case i_position_feedback[int i].exit():
                //                loop_flag = 0;
                continue;

        case tx when timerafter(time1+2500) :> void: //12 usec 3000

                if(++hall_last_state_period  > HALL_TRANSITION_PERIOD_MAX)  hall_last_state_period=HALL_TRANSITION_PERIOD_MAX;
                if(++hall_transition_timeout > HALL_PERIOD_MAX)             hall_transition_timeout=HALL_PERIOD_MAX;

                if(++hall_period0>HALL_PERIOD_MAX)  hall_period0=HALL_PERIOD_MAX;
                if(++hall_period1>HALL_PERIOD_MAX)  hall_period1=HALL_PERIOD_MAX;
                if(++hall_period2>HALL_PERIOD_MAX)  hall_period2=HALL_PERIOD_MAX;
                if(++hall_period3>HALL_PERIOD_MAX)  hall_period3=HALL_PERIOD_MAX;
                if(++hall_period4>HALL_PERIOD_MAX)  hall_period4=HALL_PERIOD_MAX;
                if(++hall_period5>HALL_PERIOD_MAX)  hall_period5=HALL_PERIOD_MAX;


                switch(hall_stable_states)
                {
                case 0:
                    hall_ports.p_hall :> hall_state_1;
                    hall_state_1 &= 0x07;
                    hall_ports.p_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_1 == hall_state_2)
                        hall_stable_states++;
                    break;


                case 1:
                    hall_ports.p_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_2 == hall_state_1)
                        hall_stable_states++;
                    else
                        hall_stable_states=0;
                    break;


                case 2:
                    hall_ports.p_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_2 == hall_state_1)
                        hall_state_new = hall_state_2;
                    else
                        hall_stable_states=0;
                    break;

                default:
                    break;
                }


                if(hall_state_new != hall_state_old)
                {
                    tx :> hall_new_change_moment;

                    hall_state_old = hall_state_new;

                    hall_transition_timeout = 0;

                    if(hall_state_new == hall_state_next)
                    {
                        hv.hall_position++;
                        if(hall_direction < 1)  hall_direction++;
                    }

                    if(hall_state_new == hall_state_previous)
                    {
                        hv.hall_position--;

                        if(hall_direction > -1) hall_direction--;
                    }

                    switch(hall_state_new)
                    {
                    case HALL_STATE_0:
                        hall_state_next     = HALL_STATE_1;
                        hall_state_previous = HALL_STATE_5;
                        hall_sector_and_state = 0x80 + hall_state_new;
                        hall_last_period = hall_period0;
                        hall_period0 = 0;
                        break;

                    case HALL_STATE_1:
                        hall_state_next     = HALL_STATE_2;
                        hall_state_previous = HALL_STATE_0;
                        hall_sector_and_state = 0x10 + hall_state_new;
                        hall_last_period = hall_period1;
                        hall_period1 = 0;
                        break;

                    case HALL_STATE_2:
                        hall_state_next     = HALL_STATE_3;
                        hall_state_previous = HALL_STATE_1;
                        hall_sector_and_state = 0x20 + hall_state_new;
                        hall_last_period = hall_period2;
                        hall_period2 = 0;
                        break;

                    case HALL_STATE_3:
                        hall_state_next     = HALL_STATE_4;
                        hall_state_previous = HALL_STATE_2;
                        hall_sector_and_state = 0x30 + hall_state_new;
                        hall_last_period = hall_period3;
                        hall_period3 = 0;
                        break;

                    case HALL_STATE_4:
                        hall_state_next     = HALL_STATE_5;
                        hall_state_previous = HALL_STATE_3;
                        hall_sector_and_state = 0x40 + hall_state_new;
                        hall_last_period = hall_period4;
                        hall_period4 = 0;
                        break;

                    case HALL_STATE_5:
                        hall_state_next     = HALL_STATE_0;
                        hall_state_previous = HALL_STATE_4;
                        hall_sector_and_state = 0x50 + hall_state_new;
                        hall_last_period = hall_period5;
                        hall_period5 = 0;
                        break;

                    default:
                        hall_error++;
                        break;
                    }

                    hall_last_state_period = 0;

                    hall_transition_time = (hall_new_change_moment - hall_previous_change_moment);
                    hall_previous_change_moment = hall_new_change_moment;


                    if(hall_last_period >= HALL_PERIOD_MAX)
                    {
                        hv.hall_period = HALL_PERIOD_MAX;

                        hall_sector_number_and_state_temp = hall_sector_and_state;
                        hall_transient_period_temp = hall_transition_time;
                    }
                    else
                    {

                        hv.hall_period = hall_last_period;

                        hall_sector_number_and_state_temp = hall_sector_and_state;
                        hall_transient_period_temp = hall_transition_time;

                        hall_sector_and_state = 0;

                        hall_position_temp = hv.hall_position;
                    }

                    hv.hall_transition_period = hall_transient_period_temp;

                    hall_sector_and_state_temp = hall_sector_number_and_state_temp;


                    if(hall_sector_and_state_temp)
                    {
                        sector_transition(hv,hall_sector_and_state_temp);
                    }
                    else
                    {
                        if(hv.hall_last_transition_period >= HALL_TRANSITION_PERIOD_MAX)         hv.hall_speed = 0;
                    }


                    hall_calculate_speed(hv);
                    speed_LPF(hv);


                    hv.hall_last_transition_period=hall_last_state_period;
                    hall_calculate_angle(hv);

                    hv.hall_speed_before_stopping = hv.hall_filtered_speed;
                }
                else
                {
                    tx :> hall_waiting_time;
                    hall_transition_period_at_low_speeds = hall_waiting_time - hall_previous_change_moment;
                    if (hall_transition_period_at_low_speeds > (199*(hall_transition_time/100)))
                    {
                        speed_reduction_factor = hall_transition_time / hall_transition_period_at_low_speeds;
                        hv.hall_filtered_speed = hv.hall_speed_before_stopping * speed_reduction_factor;
                    }

                }

                hv.hall_last_transition_period=hall_last_state_period;

                hall_calculate_angle(hv);


                //                    i_update_control_position.update_control_angle(hv.hall_interpolated_angle, hv.hall_filtered_speed, hall_state_new);



                angle_out = hv.hall_interpolated_angle;
                speed_out = hv.hall_filtered_speed;

                if (hv.sensor_polarity==-1)//inverted polarity
                {
                    angle_out = 4095 - hv.hall_interpolated_angle;
                    speed_out = -hv.hall_filtered_speed;
                }

                if(angle_out>4095) angle_out-=4096;
                if(angle_out<0)    angle_out+=4096;

                if (init_angle) {
                    multiturn(count, last_angle, angle_out, 4096);
                } else {
                    init_angle = 1;
                }
                last_angle = angle_out;

                i_shared_memory.write_angle_velocity_position_hall(angle_out, speed_out, count, hall_state_new);






















                //                switch (xreadings) {
                //                    case 0:
                    //                        hall_ports.p_hall :> new1;
                    //                        new1 &= 0x07;
                    //                        xreadings++;
                    //                        break;
                    //                    case 1:
                        //                        hall_ports.p_hall :> new2;
                        //                        new2 &= 0x07;
                        //                        if (new2 == new1) {
                            //                            xreadings++;
                            //                        } else {
                                //                            xreadings=0;
                                //                        }
                //                        break;
                //                    case 2:
                //                        hall_ports.p_hall :> new2;
                //                        new2 &= 0x07;
                //                        if (new2 == new1) {
                //                            pin_state = new2;
                //                        } else {
                //                            xreadings=0;
                //                        }
                //                        break;
                //                }//eof switch
                //
                //                hall_ports.p_hall :> pin_state_monitor;
                //                pin_state_monitor &= 0x07;
                //
                //                iCountMicroSeconds = iCountMicroSeconds + PULL_PERIOD_USEC; // period in 12 usec
                //                iTimeCountOneTransition = iTimeCountOneTransition + PULL_PERIOD_USEC ;
                //
                //                if (pin_state != pin_state_last) {
                //                    if(pin_state == uHallNext) {
                //                        direction = 1;
                //                    }
                //
                //                    if (pin_state == uHallPrevious) {
                //                        direction =-1;
                //                    }
                //
                //                    //if(direction >= 0) // CW  3 2 6 4 5 1
                //
                //                    switch(pin_state) {
                //                        case 3:
                //                            angle1 = 0;
                //                            uHallNext = 2;
                //                            uHallPrevious = 1;
                //                            break;
                //                        case 2:
                //                            angle1 = 682;
                //                            uHallNext = 6;
                //                            uHallPrevious = 3;
                //                            break; //  60
                //                        case 6:
                //                            angle1 = 1365;
                //                            uHallNext = 4;
                //                            uHallPrevious = 2;
                //                            break;
                //                        case 4:
                //                            angle1 = 2048;
                //                            uHallNext = 5;
                //                            uHallPrevious = 6;
                //                            break; // 180
                //                        case 5:
                //                            angle1 = 2730;
                //                            uHallNext = 1;
                //                            uHallPrevious = 4;
                //                            break;
                //                        case 1:
                //                            angle1 = 3413;
                //                            uHallNext = 3;
                //                            uHallPrevious = 5;
                //                            break; // 300 degree
                //                        default:
                //                            iHallError++;
                //                            break;
                //                    }
                //
                //                    if (direction == 1)
                //                        if (pin_state_last == 1 && pin_state == 3) {
                //                            // transition to NULL
                //                            iPeriodMicroSeconds = iCountMicroSeconds;
                //                            iCountMicroSeconds = 0;
                //                            if (iPeriodMicroSeconds) {
                //                                time_elapsed = iPeriodMicroSeconds;
                //                            }
                //                        }
                //
                //                    if (direction == -1) {
                //                        if (pin_state_last == 3 && pin_state == 1) {
                //                            iPeriodMicroSeconds = iCountMicroSeconds;
                //                            iCountMicroSeconds = 0;
                //                            if (iPeriodMicroSeconds) {
                //                                time_elapsed = 0 - iPeriodMicroSeconds;
                //                            }
                //                        }
                //                    }
                //
                //                    iTimeSaveOneTransition = iTimeCountOneTransition;
                //                    iTimeCountOneTransition = 0;
                //                    delta_angle = 0;
                //                    pin_state_last = pin_state;
                //
                //                }// end (pin_state != pin_state_last
                //
                //
                //                if (iCountMicroSeconds > defPeriodMax) {
                //                    iCountMicroSeconds = defPeriodMax;
                //                }
                //
                //                if (iTimeSaveOneTransition) {
                //                    delta_angle = (682 * iTimeCountOneTransition) / iTimeSaveOneTransition;
                //                }
                //
                //                if (delta_angle >= 680) {
                //                    delta_angle = 680;
                //                }
                //
                //                if (iTimeCountOneTransition > 50000) {
                //                    direction = 0;
                //                }
                //
                //                angle = angle1;
                //
                //                if (direction == 1) {
                //                    angle += delta_angle;
                //                }
                //
                //                if (direction == -1) {
                //                    angle -= delta_angle;
                //                }
                //
                //                angle &= 0x0FFF; // 4095
                //
                //                if (first == 1) {
                //                    previous_position = angle;
                //                    first = 0;
                //                }
                //
                //                if (previous_position != angle) {
                //                    position = angle;
                //                    if (position - previous_position <= -1800) {
                //                        count = count + (4095 + position - previous_position);
                //                    } else if (position - previous_position >= 1800) {
                //                        count = count + (-4095 + position - previous_position);
                //                    } else {
                //                        count = count + position - previous_position;
                //                    }
                //                    previous_position = angle;
                //                }
                //
                //                if (count > config_hall_max_count || count < config_hall_min_count) {
                //                    count = 0;
                //                }
                //
                //                if (status == 1) {
                //                     status = 0;
                //                }
                //
                //                if (!isnull(i_shared_memory)) {
                //                    if (position_feedback_config.hall_config.enable_push_service == PushAll) {
                //                        i_shared_memory.write_angle_velocity_position(angle, raw_velocity, count);
                //                    } else if (position_feedback_config.hall_config.enable_push_service == PushAngle) {
                //                        i_shared_memory.write_angle_electrical(angle);
                //                    } else if (position_feedback_config.hall_config.enable_push_service == PushPosition) {
                //                        i_shared_memory.write_velocity_position(raw_velocity, count);
                //                    }
                //                }

                tx :> time1;
                break;

                //            case tx when timerafter(time1 + MSEC_FAST) :> time1:
                //                if (init_velocity == 0) {
                //                    if (count > 2049) {
                //                        init_velocity = 1;
                //                        previous_position1 = 2049;
                //                    } else if (count < -2049) {
                //                        init_velocity = 1;
                //                        previous_position1 = -2049;
                //                    }
                //                    velocity = 0;
                //                } else {
                //                    difference1 = count - previous_position1;
                //                    if (difference1 > hall_crossover) {
                //                        difference1 = old_difference;
                //                    } else if (difference1 < -hall_crossover) {
                //                        difference1 = old_difference;
                //                    }
                //                    velocity = difference1;
                //                    previous_position1 = count;
                //                    old_difference = difference1;
                //                }
                //FIXME check if the velocity computation is disturbing the position measurement
                //                raw_velocity = _modified_internal_filter(filter_buffer, index, filter_length, velocity);
                //                raw_velocity = filter(filter_buffer, index, filter_length, velocity);
                // velocity in rpm = ( difference ticks * (1 minute / 1 ms) ) / ticks per turn
                //                 = ( difference ticks * (60,000 ms / 1 ms) ) / ticks per turn
                //                raw_velocity = (raw_velocity * 60000 ) / config_max_ticks_per_turn;
                //                break;

        }
        //#pragma xta endpoint "hall_loop_stop"
    }
}






void sector_transition(hall_variables & hv, int hall_sector_and_state)
{

    hv.hall_sector = (hall_sector_and_state & 0xF0);

    if(hv.hall_sector<0x80)
    {
        hv.hall_sector /= 16;
        hv.hall_sector |= 0x40;
    }

    hv.hall_pin_state = hall_sector_and_state & 0x07;

    if(hv.hall_pin_state == hv.hall_next_state)     hv.hall_direction_of_rotation =  1;
    if(hv.hall_pin_state == hv.hall_previous_state) hv.hall_direction_of_rotation = -1;

    switch(hv.hall_pin_state)
    {

    case HALL_STATE_0:
        if(hv.hall_next_state == HALL_STATE_0)
            hv.hall_angle = HALL_ANGLE_0;
        else
            hv.hall_angle = HALL_ANGLE_1;
        hv.hall_next_state = HALL_STATE_1;
        hv.hall_previous_state = HALL_STATE_5;
        break;

    case HALL_STATE_1:
        if(hv.hall_next_state == HALL_STATE_1)
            hv.hall_angle = HALL_ANGLE_1;
        else
            hv.hall_angle = HALL_ANGLE_2;
        hv.hall_next_state = HALL_STATE_2;
        hv.hall_previous_state = HALL_STATE_0;
        break;

    case HALL_STATE_2:
        if(hv.hall_next_state == HALL_STATE_2)
            hv.hall_angle = HALL_ANGLE_2;
        else
            hv.hall_angle = HALL_ANGLE_3;
        hv.hall_next_state = HALL_STATE_3;
        hv.hall_previous_state = HALL_STATE_1;
        break;

    case HALL_STATE_3:
        if(hv.hall_next_state == HALL_STATE_3)
            hv.hall_angle = HALL_ANGLE_3;
        else
            hv.hall_angle = HALL_ANGLE_4;
        hv.hall_next_state = HALL_STATE_4;
        hv.hall_previous_state = HALL_STATE_2;
        break;

    case HALL_STATE_4:
        if(hv.hall_next_state == HALL_STATE_4)
            hv.hall_angle = HALL_ANGLE_4;
        else
            hv.hall_angle = HALL_ANGLE_5;
        hv.hall_next_state = HALL_STATE_5;
        hv.hall_previous_state = HALL_STATE_3;
        break;

    case HALL_STATE_5:
        if(hv.hall_next_state == HALL_STATE_5)
            hv.hall_angle = HALL_ANGLE_5;
        else
            hv.hall_angle = HALL_ANGLE_0;
        hv.hall_next_state = HALL_STATE_0;
        hv.hall_previous_state = HALL_STATE_4;
        break;
    }
}


void hall_calculate_speed(hall_variables& hv)
{

    // rejecting abnormal conditions
    if((hv.hall_transition_period<2)||(hv.hall_sector>=0xF0))
    {
        hv.hall_speed = 0;
        return;
    }

    // speed calculation under normal conditions
    if(hv.hall_sector & 0xF0)
    {
        hv.hall_speed = hv.hall_transition_period_at_1rpm / (hv.hall_transition_period);
        if(hv.hall_direction_of_rotation == -1) hv.hall_speed = - hv.hall_speed;
    }
}



void hall_calculate_angle(hall_variables & hv)
{

    if(hv.hall_last_transition_period<(hv.hall_period/6))
    {
        hv.hall_increment = hv.hall_last_transition_period *4096;
        hv.hall_increment /= hv.hall_period;

        if(hv.hall_direction_of_rotation == 1)   hv.hall_interpolated_angle = hv.hall_angle + hv.hall_increment;
        if(hv.hall_direction_of_rotation == -1)  hv.hall_interpolated_angle = hv.hall_angle - hv.hall_increment;
    }
    else
    {
        hv.hall_increment = 0;
    }

    if (hv.hall_interpolated_angle > 4095)   hv.hall_interpolated_angle -= 4096;
    if (hv.hall_interpolated_angle <    0)   hv.hall_interpolated_angle += 4096;
}



void speed_LPF(hall_variables& hv)
{
    int k;
    int y;
    int speed_index=0;

    ++hv.hall_filter_index_newest;

    if(hv.hall_filter_index_newest == hv.hall_filter_order) hv.hall_filter_index_newest = 0;

    hv.hall_filter_buffer[hv.hall_filter_index_newest] = hv.hall_speed;
    y = 0;

    speed_index = hv.hall_filter_index_newest;
    for(k=0;k<hv.hall_filter_order;k++)
    {
        y += hv.h[k] * hv.hall_filter_buffer[speed_index];
        --speed_index;
        if(speed_index == -1)   speed_index = hv.hall_filter_order -1;
    }

    hv.hall_filtered_speed = y / 1000;

}



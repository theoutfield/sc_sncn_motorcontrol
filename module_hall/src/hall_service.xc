/**
 * @file hall_server.xc
 * @brief Hall Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <hall_service.h>
#include <print.h>
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

void hall_service(HallPorts &hall_ports, PositionFeedbackConfig &position_feedback_config,
                  client interface shared_memory_interface ?i_shared_memory,
                  server interface PositionFeedbackInterface i_position_feedback[3])
{

    if (HALL_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    if (check_hall_config(position_feedback_config.hall_config) == ERROR) {
        printstrln("hall_service: ERROR: Error while checking the Hall sensor configuration");
        position_feedback_config.sensor_type = 0;
        return;
    }

    printstr(">>   SOMANET HALL SENSOR SERVICE STARTING...\n");

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

    int notification = MOTCTRL_NTF_EMPTY;


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
    hv.hall_f_clock = (HALL_USEC*1000000); //1 second in ticks
    // motor pole pairs
    hv.hall_pole_pairs = position_feedback_config.hall_config.pole_pairs;
    hv.hall_transition_period_at_1rpm = (hv.hall_f_clock / (hv.hall_pole_pairs*6)) * 60 ;

    hv.sensor_polarity=position_feedback_config.hall_config.polarity;


    do
    {
        hall_ports.p_hall :> hall_state_1;
        hall_ports.p_hall :> hall_state_2;
    } while(hall_state_1 != hall_state_2);

    hall_state_new = hall_state_1;
    hall_state_old = hall_state_1;

    tx :> time1;

    int loop_flag = 1;
    while (loop_flag) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        case i_position_feedback[int i].get_angle() -> unsigned int out_angle:
                out_angle = angle_out;
                break;

        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int out_position }:
                out_count = count;
                out_position = angle_out;
                break;

        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = speed_out;
                break;

        case i_position_feedback[int i].set_position(int in_count):
                count = in_count;
                last_angle = angle_out;
                break;

        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                position_feedback_config = in_config;
                hv.hall_pole_pairs = position_feedback_config.hall_config.pole_pairs;
                hv.hall_transition_period_at_1rpm = (hv.hall_f_clock / (hv.hall_pole_pairs*6)) * 60 ;
                hv.sensor_polarity=position_feedback_config.hall_config.polarity;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }
                break;

        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                out_ticks_per_turn = HALL_TICKS_PER_ELECTRICAL_ROTATION;
                break;

        case i_position_feedback[int i].set_angle(unsigned int in_angle) -> unsigned int out_offset:
                break;

        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int out_position,  unsigned int out_status}:
                break;

        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int out_status:
                break;

        case i_position_feedback[int i].exit():
                loop_flag = 0;
                continue;

        case tx when timerafter(time1+(HALL_USEC*10)) :> void: //10 usec

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
                    multiturn(count, last_angle, angle_out, HALL_TICKS_PER_ELECTRICAL_ROTATION);
                } else {
                    init_angle = 1;
                }
                last_angle = angle_out;

                if (!isnull(i_shared_memory)) {
                    if (position_feedback_config.contelec_config.enable_push_service == PushAll) {
                        i_shared_memory.write_angle_velocity_position_hall(angle_out, speed_out, count, hall_state_new);
                    } else if (position_feedback_config.contelec_config.enable_push_service == PushAngle) {
                        i_shared_memory.write_angle_electrical(angle_out);
                    } else if (position_feedback_config.contelec_config.enable_push_service == PushPosition) {
                        i_shared_memory.write_velocity_position(speed_out, count);
                    }
                }


                tx :> time1;
                break;

        }
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

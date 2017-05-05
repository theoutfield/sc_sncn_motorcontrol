/**
 * @file hall_server.xc
 * @brief Hall Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <hall_service.h>
#include <print.h>
#include <mc_internal_constants.h>

extern char start_message[];

int check_hall_config(PositionFeedbackConfig &position_feedback_config){

    if (position_feedback_config.pole_pairs < 1) {
        printstrln("hall_service: ERROR: Wrong Hall configuration: wrong pole-pairs");
        return HALL_ERROR;
    }

    return HALL_SUCCESS;
}

void sector_transition(hall_variables& hv, int pin_state);
void hall_calculate_speed(hall_variables& hv);
void hall_calculate_angle(hall_variables& hv);
void speed_LPF(hall_variables& hv);


//lookup tables for hall states and angle transitions
//hall state value to hall state number
static const unsigned int hall_state_reverse[6] = {
        4,
        2,
        3,
        0,
        5,
        1
};
//hall state to hall state next
static const unsigned int hall_state_next[6] = {
        HALL_STATE_5,
        HALL_STATE_3,
        HALL_STATE_4,
        HALL_STATE_1,
        HALL_STATE_0,
        HALL_STATE_2
};
//hall state to hall state previous
static const unsigned int hall_state_prev[6] = {
        HALL_STATE_3,
        HALL_STATE_1,
        HALL_STATE_2,
        HALL_STATE_5,
        HALL_STATE_4,
        HALL_STATE_0
};
//hall state to to hall angle with same number
static const unsigned int hall_angle[6] = {
        HALL_ANGLE_4,
        HALL_ANGLE_2,
        HALL_ANGLE_3,
        HALL_ANGLE_0,
        HALL_ANGLE_5,
        HALL_ANGLE_1
};
//hall state to hall angle next number
static const unsigned int hall_angle_next[6] = {
        HALL_ANGLE_5,
        HALL_ANGLE_3,
        HALL_ANGLE_4,
        HALL_ANGLE_1,
        HALL_ANGLE_0,
        HALL_ANGLE_2
};
//hall state value to hall half angle
static const unsigned int hall_half_angle[6] = {
        (HALL_ANGLE_4+HALL_ANGLE_5)/2,
        (HALL_ANGLE_2+HALL_ANGLE_3)/2,
        (HALL_ANGLE_3+HALL_ANGLE_4)/2,
        1,
        (HALL_ANGLE_5+HALL_ANGLE_0)/2,
        (HALL_ANGLE_1+HALL_ANGLE_2)/2
};

void hall_service(QEIHallPort &qei_hall_port, port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
        client interface shared_memory_interface ?i_shared_memory,
                server interface PositionFeedbackInterface i_position_feedback[3])
{

#ifdef DEBUG_POSITION_FEEDBACK
    if (check_hall_config(position_feedback_config) != HALL_SUCCESS) {
        printstrln("hall_service: ERROR: Error while checking the Hall sensor configuration");
        position_feedback_config.sensor_type = 0;
    }

    printstr(start_message);
    printstrln("HALL");
#endif

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

    unsigned int hall_state_next_local;
    unsigned int hall_state_previous;

    unsigned int hall_transition_timeout;
    hall_transition_timeout=HALL_PERIOD_MAX;

    unsigned int hall_last_period;
    unsigned int hall_period[6];

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
    hv.hall_f_clock = (position_feedback_config.ifm_usec*1000000); //1 second in ticks
    // motor pole pairs
    hv.hall_pole_pairs = position_feedback_config.pole_pairs;
    hv.hall_transition_period_at_1rpm = (hv.hall_f_clock / (hv.hall_pole_pairs*6)) * 60 ;

    hv.sensor_polarity=position_feedback_config.polarity;

    do
    {
        qei_hall_port.p_qei_hall :> hall_state_1;
        qei_hall_port.p_qei_hall :> hall_state_2;
    } while(hall_state_1 != hall_state_2);

    hall_state_new = hall_state_1;
    hall_state_old = hall_state_1;

    tx :> time1;

    int loop_flag = 1;
    while (loop_flag)
    {
        select
        {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        case i_position_feedback[int i].get_angle() -> unsigned int out_angle:
                out_angle = angle_out;
                break;

        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int out_position, SensorError status }:
                out_count = count;
                out_position = angle_out;
                status = SENSOR_NO_ERROR;
                break;

        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = speed_out;
                break;

        case i_position_feedback[int i].set_position(int in_count):
                count = in_count + position_feedback_config.offset;
                last_angle = angle_out;
                break;

        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                UsecType ifm_usec = position_feedback_config.ifm_usec;
                position_feedback_config = in_config;
                position_feedback_config.ifm_usec = ifm_usec;
                hv.hall_pole_pairs = position_feedback_config.pole_pairs;
                hv.hall_transition_period_at_1rpm = (hv.hall_f_clock / (hv.hall_pole_pairs*6)) * 60 ;
                hv.sensor_polarity=position_feedback_config.polarity;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }
                break;

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

        case tx when timerafter(time1+(position_feedback_config.ifm_usec*10)) :> void: //10 usec

                if(++hall_last_state_period  > HALL_TRANSITION_PERIOD_MAX)  hall_last_state_period=HALL_TRANSITION_PERIOD_MAX;
                if(++hall_transition_timeout > HALL_PERIOD_MAX)             hall_transition_timeout=HALL_PERIOD_MAX;

                for (int i=0; i<6; i++) {
                    if(++hall_period[i]>HALL_PERIOD_MAX)  hall_period[i]=HALL_PERIOD_MAX;
                }


                switch(hall_stable_states)
                {
                case 0:
                    qei_hall_port.p_qei_hall :> hall_state_1;
                    hall_state_1 &= 0x07;
                    qei_hall_port.p_qei_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_1 == hall_state_2)
                        hall_stable_states++;
                    break;


                case 1:
                    qei_hall_port.p_qei_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_2 == hall_state_1)
                        hall_stable_states++;
                    else
                        hall_stable_states=0;
                    break;


                case 2:
                    qei_hall_port.p_qei_hall :> hall_state_2;
                    hall_state_2 &= 0x07;
                    if(hall_state_2 == hall_state_1)
                        hall_state_new = hall_state_2;
                    else
                        hall_stable_states=0;
                    break;
                }


                if(hall_state_new != hall_state_old)
                {
                    tx :> hall_new_change_moment;

                    hall_state_old = hall_state_new;

                    hall_transition_timeout = 0;

                    if(hall_state_new == hall_state_next_local)
                    {
                        hv.hall_position++;
                        if(hall_direction < 1)  hall_direction++;
                    }

                    if(hall_state_new == hall_state_previous)
                    {
                        hv.hall_position--;

                        if(hall_direction > -1) hall_direction--;
                    }


                    if (hall_state_new >= 1 && hall_state_new <= 6)
                    {
                        hall_state_next_local = hall_state_next[hall_state_new-1];
                        hall_state_previous = hall_state_prev[hall_state_new-1];
                        if (hall_state_new == HALL_STATE_0) {
                            hall_sector_and_state = 0x80 + HALL_STATE_0;
                        } else {
                            hall_sector_and_state = hall_state_reverse[hall_state_new-1]*0x10 + hall_state_new;
                        }
                        hall_last_period = hall_period[hall_state_new-1];
                        hall_period[hall_state_new-1] = 0;
                    }
                    else
                    {
                        hall_error++;
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
                        if(hv.hall_last_transition_period >= HALL_TRANSITION_PERIOD_MAX)
                        {
                            hv.hall_speed = 0;

                            if (hv.hall_pin_state >=1 && hv.hall_pin_state <= 6) {
                                hv.hall_angle = hall_half_angle[hv.hall_pin_state-1];
                            }
                        }
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

                if (position_feedback_config.polarity==SENSOR_POLARITY_INVERTED)//inverted polarity
                {
                    angle_out = 4095 - angle_out;
                    speed_out = -speed_out;
                }

                if (init_angle) {
                    multiturn(count, last_angle, angle_out, HALL_TICKS_PER_ELECTRICAL_ROTATION);
                } else {
                    init_angle = 1;
                }
                last_angle = angle_out;

                tx :> time1;
                write_shared_memory(i_shared_memory, position_feedback_config.sensor_function, count + position_feedback_config.offset, angle_out, speed_out, angle_out, hall_state_new, SENSOR_NO_ERROR, SENSOR_NO_ERROR, time1/position_feedback_config.ifm_usec);

                //gpio
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory);


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


    if (hv.hall_pin_state >= 1 && hv.hall_pin_state <= 6) {
        if(hv.hall_next_state == hv.hall_pin_state)
            hv.hall_angle = hall_angle[hv.hall_pin_state-1];
        else
            hv.hall_angle = hall_angle_next[hv.hall_pin_state-1];
        hv.hall_next_state = hall_state_next[hv.hall_pin_state-1];
        hv.hall_previous_state = hall_state_prev[hv.hall_pin_state-1];
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
    hv.hall_filter_index_newest = (hv.hall_filter_index_newest+1) % hv.hall_filter_order;

    hv.hall_filter_buffer[hv.hall_filter_index_newest] = hv.hall_speed;

    int y = 0;
    int speed_index = hv.hall_filter_index_newest;
    for(int k=0;k<hv.hall_filter_order;k++)
    {
        y += hv.h[k] * hv.hall_filter_buffer[speed_index];
        --speed_index;
        if(speed_index == -1)   speed_index = hv.hall_filter_order -1;
    }

    hv.hall_filtered_speed = y / 1000;

}

/**
 * @file qei_service.xc
 * @brief Incremental Encoder Service Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <qei_service.h>
#include <limits.h>
#include <print.h>
#include <mc_internal_constants.h>
#include <filters.h>

// Order is 00 -> 10 -> 11 -> 01
// Bit 3 = Index
static const signed char lookup[4][4] = {
    {  0, -1,  1,  0 }, // 00
    {  1,  0,  0, -1 }, // 01
    { -1,  0,  0,  1 }, // 10
    {  0,  1, -1,  0 }, // 11
};

extern char start_message[];

int check_qei_config(PositionFeedbackConfig &position_feedback_config)
{
    if (position_feedback_config.qei_config.number_of_channels != QEI_WITH_INDEX && position_feedback_config.qei_config.number_of_channels != QEI_WITH_NO_INDEX) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong index type");
        return QEI_ERROR;
    }

    if (position_feedback_config.polarity != SENSOR_POLARITY_INVERTED && position_feedback_config.polarity != SENSOR_POLARITY_NORMAL) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong polarity");
        return QEI_ERROR;
    }

    if (position_feedback_config.resolution <= 0) {
        printstrln("qei_service: ERROR: Wrong QEI configuration: wrong resolution");
        return QEI_ERROR;
    }

    return QEI_SUCCESS;
}

void qei_service(port qei_hall_port, port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
                 client interface shared_memory_interface ?i_shared_memory,
                 server interface PositionFeedbackInterface i_position_feedback[3],
                 int gpio_on)
{

#ifdef DEBUG_POSITION_FEEDBACK
    if (check_qei_config(position_feedback_config) != QEI_SUCCESS) {
        position_feedback_config.sensor_type = 0;
    }

    printstr(start_message);
    printstrln("QEI");
#endif

    int position = 0;
    int count = 0;
    unsigned int angle = 0;
    unsigned int index_found =0;

    unsigned int old_pins = 0;
    unsigned int new_pins;
    unsigned int new_pins_1;

    int qei_crossover_velocity = position_feedback_config.resolution - position_feedback_config.resolution / 10;
    int vel_previous_position = 0;
    int velocity = 0;
    int velocity_ratio = (60000000/(position_feedback_config.velocity_compute_period*position_feedback_config.resolution));
//    int velocity_buffer [8] = {0};
//    int index = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    SensorError sensor_error = SENSOR_NO_ERROR, last_sensor_error = SENSOR_NO_ERROR;
    int singleturn_counter = 0, singleturn_old = 0, ticks_lost;

    // used to compute the singleturn
    int shift = position_feedback_config.resolution;
    int shift_counter = 0;
    while (!shift%2 && shift_counter < 16)
    {
        shift /= 2;
        shift_counter ++;
    }

    timer t_velocity;
    unsigned int ts_velocity;

    UsecType ifm_usec = position_feedback_config.ifm_usec;

    t_velocity :> ts_velocity;

    qei_hall_port :> new_pins;

    int loop_flag = 1;
    while (loop_flag) {
        select {
            case qei_hall_port when pinsneq(new_pins) :> new_pins :
                qei_hall_port :> new_pins_1;
                qei_hall_port :> new_pins_1;
                if (new_pins_1 == new_pins) {
                    qei_hall_port :> new_pins;
                    if (new_pins_1 == new_pins) {
                        int inc;
                        if (position_feedback_config.polarity == SENSOR_POLARITY_NORMAL) {
                            inc =  lookup[new_pins&0b11][old_pins];
                        } else {
                            inc = -lookup[new_pins&0b11][old_pins];
                        }
                        old_pins = new_pins & 0b11;

                        count += inc;
                        position += inc;

                        // set positon to positive
                        if (position < 0) {
                            position += position_feedback_config.resolution;
                        }

                        //index
                        if ((position_feedback_config.qei_config.number_of_channels == QEI_WITH_INDEX) && (new_pins & 0b0100)) {
                            position = 0;
                            if (index_found == 0) {
                                count = 0; //reset count the first time we find the index
                                index_found = 1;
                            }

                            //realign the count with the position in case we missed ticks
                            int align = (count % position_feedback_config.resolution);
                            if (align != 0)
                            {
                                if (align > 0)
                                    ticks_lost = position_feedback_config.resolution - align;
                                else
                                    ticks_lost = position_feedback_config.resolution + align;

                                // generate an error
                                if(ticks_lost > position_feedback_config.resolution/position_feedback_config.qei_config.ticks_lost_threshold)
                                    sensor_error = SENSOR_QEI_INDEX_LOSING_TICKS;
                            }


                            if ((align < (position_feedback_config.resolution/2)) && (align > (-position_feedback_config.resolution/2))) {
                                count -= align;
                            } else if (align > 0) {
                                count += (position_feedback_config.resolution - align);
                            } else if (align < 0) {
                                count += (-position_feedback_config.resolution - align);
                            }
                        }

                        // reset position if bigger than resolution
                        // this also takes care of the case of QEI without index
                        if (position >= position_feedback_config.resolution) {
                            position = 0;
                        }

                        angle = ((position * position_feedback_config.pole_pairs* (1<<12))/ position_feedback_config.resolution) & ((1<<12)-1);
                        unsigned int timestamp;
                        t_velocity :> timestamp;
                        write_shared_memory(i_shared_memory, position_feedback_config.sensor_function, count + position_feedback_config.offset, position * (1 << (16 - shift_counter)) / shift, velocity, angle, 0, index_found, sensor_error, last_sensor_error, timestamp/ifm_usec);
                    }
                }
                break;

            case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

            case i_position_feedback[int i].get_position() -> { int out_count, unsigned int out_position, SensorError status }:
                out_count = count + position_feedback_config.offset;
                out_position = position;
                status = sensor_error;
                break;

            case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

            case i_position_feedback[int i].set_position(int in_count):
                 count = in_count;
                 break;

            case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

            case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                sensor_error = SENSOR_NO_ERROR;
                last_sensor_error = SENSOR_NO_ERROR;
                singleturn_counter = 0;
                position_feedback_config = in_config;
                position_feedback_config.ifm_usec = ifm_usec;
                shift = position_feedback_config.resolution;
                shift_counter = 0;
                while (!shift%2 && shift_counter < 16)
                {
                    shift /= 2;
                    shift_counter ++;
                }
                qei_crossover_velocity = position_feedback_config.resolution - position_feedback_config.resolution / 10;
                velocity_ratio = (60000000/(position_feedback_config.velocity_compute_period*position_feedback_config.resolution));
                write_hall_state_angle_shared_memory(position_feedback_config, i_shared_memory);
                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }
                break;

            case i_position_feedback[int i].get_angle() -> unsigned int out_angle:
                out_angle = angle;
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

            case t_velocity when timerafter(ts_velocity + (position_feedback_config.velocity_compute_period*ifm_usec)) :> ts_velocity:
                int difference_velocity = count - vel_previous_position;
                if (difference_velocity < qei_crossover_velocity && difference_velocity > -qei_crossover_velocity)
                {
                    velocity = difference_velocity * velocity_ratio;
//                    velocity = filter(velocity_buffer, index, 8, velocity);
                }
                vel_previous_position = count;

                // set the singleturn position to a 16 bits format
                int singleturn = position * (1 << (16 - shift_counter)) / shift;

                if(singleturn != singleturn_old)
                {
                    if (singleturn == 0)
                        singleturn_counter = 1;
                    else
                        singleturn_counter = 0;

                    singleturn_old = singleturn;
                }

                if (singleturn_counter && singleturn == 0)
                    ++singleturn_counter;

                if (singleturn_counter == 1000)
                    sensor_error = SENSOR_INCREMENTAL_FAULT;

                if (sensor_error != SENSOR_NO_ERROR)
                    last_sensor_error = sensor_error;

                write_shared_memory(i_shared_memory, position_feedback_config.sensor_function, count + position_feedback_config.offset, singleturn,  velocity, angle, 0, index_found, sensor_error, last_sensor_error, ts_velocity/ifm_usec);

                //gpio
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory, gpio_on);

                break;
        }
    }
}

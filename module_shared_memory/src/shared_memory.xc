/**
 * @file shared_memory.xc
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <shared_memory.h>
#include <string.h>

[[distributable]]
void shared_memory_service(server interface shared_memory_interface i_shared_memory[n], unsigned n){

    UpstreamControlData data = {0};
    unsigned int gpio_write_buffer = 0;
    int hall_state_angle[6] = {0};
    int hall_state_angle_valid = 0;

    while (1) {
        select {
        case i_shared_memory[int j].status() -> {int status}:
                status = ACTIVE;
                break;
        case i_shared_memory[int j].gpio_write_input_read_output(unsigned int in_gpio) -> unsigned int out_gpio_write:
                out_gpio_write = gpio_write_buffer;
                for (int i=0 ; i<4 ; i++) {
                    data.gpio[i] = (in_gpio>>i)&1;
                }
                break;
        case i_shared_memory[int j].write_gpio_output(unsigned int in_gpio_write_buffer):
                gpio_write_buffer = in_gpio_write_buffer;
                break;
        case i_shared_memory[int j].read() -> UpstreamControlData out_data:
                out_data = data;
                break;

        case i_shared_memory[int j].write_angle_and_primary_feedback(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp):
                data.angle = angle;
                data.hall_state = hall_state;
                data.qei_index_found = qei_index_found;
                data.angle_velocity = velocity;
                data.angle_sensor_error = sensor_error;
                data.angle_last_sensor_error = last_sensor_error;
                data.position = position;
                data.singleturn = singleturn;
                data.velocity = velocity;
                data.sensor_error = sensor_error;
                data.last_sensor_error = last_sensor_error;
                data.sensor_timestamp = timestamp;
                break;

        case i_shared_memory[int j].write_angle(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int velocity, SensorError sensor_error, SensorError last_sensor_error):
                data.angle = angle;
                data.hall_state = hall_state;
                data.qei_index_found = qei_index_found;
                data.angle_velocity = velocity;
                data.angle_sensor_error = sensor_error;
                data.angle_last_sensor_error = last_sensor_error;
                break;

        case i_shared_memory[int j].write_angle_and_secondary_feedback(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp):
                data.angle = angle;
                data.hall_state = hall_state;
                data.qei_index_found = qei_index_found;
                data.angle_velocity = velocity;
                data.angle_sensor_error = sensor_error;
                data.angle_last_sensor_error = last_sensor_error;
                data.secondary_position = position;
                data.secondary_singleturn = singleturn;
                data.secondary_velocity = velocity;
                data.secondary_sensor_error = sensor_error;
                data.secondary_last_sensor_error = last_sensor_error;
                data.secondary_sensor_timestamp = timestamp;
                break;

        case i_shared_memory[int j].write_primary_feedback(int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp):
                data.position = position;
                data.singleturn = singleturn;
                data.velocity = velocity;
                data.sensor_error = sensor_error;
                data.last_sensor_error = last_sensor_error;
                data.sensor_timestamp = timestamp;
                break;

        case i_shared_memory[int j].write_secondary_feedback(int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp):
                data.secondary_position = position;
                data.secondary_singleturn = singleturn;
                data.secondary_velocity = velocity;
                data.secondary_sensor_error = sensor_error;
                data.secondary_last_sensor_error = last_sensor_error;
                data.secondary_sensor_timestamp = timestamp;
                break;

        case i_shared_memory[int j].write_hall_state_angle(int in_hall_state_angle[6]):
                for (int i=0 ; i<6 ; i++) {
                    hall_state_angle[i] = in_hall_state_angle[i];
                }
                hall_state_angle_valid = 1;
                break;

        case i_shared_memory[int j].read_hall_state_angle(int out_hall_state_angle[6]) -> int out_hall_state_angle_valid:
                for (int i=0 ; i<6 ; i++) {
                    out_hall_state_angle[i] = hall_state_angle[i];
                }
                out_hall_state_angle_valid = hall_state_angle_valid;
                break;
        }
    }
}

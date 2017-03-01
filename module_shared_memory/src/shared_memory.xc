/**
 * @file shared_memory.xc
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <shared_memory.h>
#include <string.h>

[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n){

    UpstreamControlData data = {0};
    unsigned int gpio_write_buffer = 0;

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

        case i_shared_memory[int j].write_angle_and_primary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity):
                data.angle = angle;
                data.hall_state = hall_state;
                data.angle_velocity = velocity;
                data.position = position;
                data.velocity = velocity;
                break;

        case i_shared_memory[int j].write_angle_and_secondary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity):
                data.angle = angle;
                data.hall_state = hall_state;
                data.angle_velocity = velocity;
                data.position_additional = position;
                data.velocity_additional = velocity;
                break;

        case i_shared_memory[int j].write_primary_feedback(int position, int velocity):
                data.position = position;
                data.velocity = velocity;
                break;

        case i_shared_memory[int j].write_secondary_feedback(int position, int velocity):
                data.position_additional = position;
                data.velocity_additional = velocity;
                break;
        }
    }
}

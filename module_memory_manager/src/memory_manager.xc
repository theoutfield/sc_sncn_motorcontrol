/**
 * @file memory_manager.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <memory_manager.h>
#include <string.h>

[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n){

    SharedMemoryData data = {0};
    unsigned int gpio_write_buffer = 0;

    while (1) {
        select {
        case i_shared_memory[int j].status() -> {int status}:
                status = ACTIVE;
                break;
        case i_shared_memory[int j].get_angle_velocity_position() -> {unsigned int out_angle, int out_velocity, int out_count}:
                out_angle = data.angle;
                out_velocity = data.velocity;
                out_count = data.position;
                break;
        case i_shared_memory[int j].get_angle_velocity_position_hall() -> {unsigned int out_angle, int out_velocity, int out_count, int out_hall}:
                out_angle = data.angle;
                out_velocity = data.velocity;
                out_count = data.position;
                out_hall = data.hall_state;
                break;
        case i_shared_memory[int j].get_angle() -> unsigned int out_angle:
                out_angle = data.angle;
                break;
        case i_shared_memory[int j].get_position_singleturn() -> unsigned position_singleturn_:
                position_singleturn_ = data.position_singleturn;
                break;
        case i_shared_memory[int j].get_position_multiturn() -> {int count_, unsigned position_singleturn_}:
                count_ = data.position;
                position_singleturn_ = data.position_singleturn;
                break;
        case i_shared_memory[int j].write_angle_electrical(int in_angle):
                data.angle = in_angle;
                break;
        case i_shared_memory[int j].write_current_velocity(int current_velocity_):
                data.velocity = current_velocity_;
                break;
        case i_shared_memory[int j].write_position_singleturn(unsigned position_singleturn_):
                data.position_singleturn = position_singleturn_;
                break;
        case i_shared_memory[int j].write_position_multiturn(int count_, unsigned position_singleturn_):
                data.position = count_;
                data.position_singleturn = position_singleturn_;
                break;
        case i_shared_memory[int j].write_angle_velocity_position(unsigned int in_angle, int in_velocity, int in_count):
                data.angle = in_angle;
                data.velocity = in_velocity;
                data.position = in_count;
                break;
        case i_shared_memory[int j].write_angle_velocity_position_hall(unsigned int in_angle, int in_velocity, int in_count, int in_hall):
                data.angle = in_angle;
                data.velocity = in_velocity;
                data.position = in_count;
                data.hall_state = in_hall;
                break;
        case i_shared_memory[int j].write_velocity_position(int in_velocity, int in_count):
                data.velocity = in_velocity;
                data.position = in_count;
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
        case i_shared_memory[int j].read() -> SharedMemoryData out_data:
                out_data = data;
                break;
        }
    }
}

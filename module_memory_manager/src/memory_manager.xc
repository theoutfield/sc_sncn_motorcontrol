/**
 * @file memory_manager.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <memory_manager.h>

[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n){
    unsigned int angle_electrical = 0;
    int current_velocity = 0;
    unsigned position_singleturn = 0;
    int multiturn_count = 0;
    int hall_state=0;
    while (1) {
        select {
        case i_shared_memory[int j].status() -> {int status}:
            status = ACTIVE;
            break;
        case i_shared_memory[int j].get_angle_velocity_position() -> {unsigned int out_angle, int out_velocity, int out_count}:
            out_angle = angle_electrical;
            out_velocity = current_velocity;
            out_count = multiturn_count;
            break;
            case i_shared_memory[int j].get_angle_velocity_position_hall() -> {unsigned int out_angle, int out_velocity, int out_count, int out_hall}:
                out_angle = angle_electrical;
                out_velocity = current_velocity;
                out_count = multiturn_count;
                out_hall = hall_state;
                break;
            case i_shared_memory[int j].get_angle() -> unsigned int out_angle:
                out_angle = angle_electrical;
                break;
            case i_shared_memory[int j].get_position_singleturn() -> unsigned position_singleturn_:
                position_singleturn_ = position_singleturn;
                break;
            case i_shared_memory[int j].get_position_multiturn() -> {int count_, unsigned position_singleturn_}:
                count_ = multiturn_count;
                position_singleturn_ = position_singleturn;
                break;
            case i_shared_memory[int j].write_angle_electrical(int in_angle):
                angle_electrical = in_angle;
                break;
            case i_shared_memory[int j].write_current_velocity(int current_velocity_):
                current_velocity = current_velocity_;
                break;
            case i_shared_memory[int j].write_position_singleturn(unsigned position_singleturn_):
                    position_singleturn_ = position_singleturn;
                    break;
            case i_shared_memory[int j].write_position_multiturn(int count_, unsigned position_singleturn_):
                    count_ = multiturn_count;
                    position_singleturn_ = position_singleturn;
                    break;
            case i_shared_memory[int j].write_angle_velocity_position(unsigned int in_angle, int in_velocity, int in_count):
                    angle_electrical = in_angle;
                    current_velocity = in_velocity;
                    multiturn_count = in_count;
                    break;
            case i_shared_memory[int j].write_angle_velocity_position_hall(unsigned int in_angle, int in_velocity, int in_count, int in_hall):
                    angle_electrical = in_angle;
                    current_velocity = in_velocity;
                    multiturn_count = in_count;
                    hall_state = in_hall;
                    break;
            case i_shared_memory[int j].write_velocity_position(int in_velocity, int in_count):
                    current_velocity = in_velocity;
                    multiturn_count = in_count;
                    break;
        }
    }
}

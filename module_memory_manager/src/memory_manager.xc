/**
 * @file memory_manager.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <memory_manager.h>

[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n){
    int angle_electrical = 0;
    int current_velocity = 0;
    unsigned position_singleturn = 0;
    int multiturn_count = 0;
    while (1) {
        select {
            case i_shared_memory[int j].get_angle_and_velocity() -> {int angle_electrical_, int current_velocity_}:
                angle_electrical_ = angle_electrical;
                current_velocity_ = current_velocity;
                break;
            case i_shared_memory[int j].get_position_singleturn() -> unsigned position_singleturn_:
                position_singleturn_ = position_singleturn;
                break;
            case i_shared_memory[int j].get_position_multiturn() -> {int count_, unsigned position_singleturn_}:
                count_ = multiturn_count;
                position_singleturn_ = position_singleturn;
                break;
            case i_shared_memory[int j].write_angle_electrical(int angle_electrical_):
                angle_electrical = angle_electrical_;
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
        }
    }
}

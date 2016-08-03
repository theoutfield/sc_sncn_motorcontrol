/**
 * @file memory_manager.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>


/**
 * @brief Type for wich data to push to the memory manager
 */
typedef enum {
    NoPush=0,
    PushAngle,
    PushPosition,
    PushAll
} PushType;


/**
 * @brief Service to exchange data between tasks OF THE SAME TILE without blocking tasks execution.
 *
 * @param Array of communication interfaces to handle n different clients
 * @param Number of supported client interfaces
 */
[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n);

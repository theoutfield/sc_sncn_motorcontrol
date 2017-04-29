/**
 * @file shared_memory.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>

/**
 * @brief Service to exchange data between tasks OF THE SAME TILE without blocking tasks execution.
 *
 * @param Array of communication interfaces to handle n different clients
 * @param Number of supported client interfaces
 */
[[distributable]]
void shared_memory_service(server interface shared_memory_interface i_shared_memory[n], unsigned n);

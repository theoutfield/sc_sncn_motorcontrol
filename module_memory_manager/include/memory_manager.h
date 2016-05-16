/**
 * @file memory_manager.h
 * @brief Memory management task for asynchronous communication
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

interface shared_memory_interface {
    /**
    * @brief Getter for electrical angle and current velocity.
    *
    * @return  Electrical angle.
    * @return  Current velocity.
    */
    {int,int} get_angle_and_velocity();

    /**
    * @brief Getter for single-turn position.
    *
    * @return  Single-turn position in ticks.
    */
    unsigned get_position_singleturn();

    /**
    * @brief Getter for multi-turn position.
    *
    * @return  Multi-turn count.
    * @return  Single-turn position in ticks.
    */
    {int, unsigned} get_position_multiturn();

    /**
    * @brief Write electrical angle to shared memory.
    *
    * @param Electrical angle.
    */
    void write_angle_electrical(int);

    /**
    * @brief Write current velocity to shared memory.
    *
    * @param Current velocity.
    */
    void write_current_velocity(int);

    /**
    * @brief Write single-turn position to shared memory.
    *
    * @param  Single-turn position in ticks.
    */
    void write_position_singleturn(unsigned);

    /**
    * @brief Write multi-turn position to shared memory.
    *
    * @param  Multi-turn count.
    * @param  Single-turn position in ticks.
    */
    void write_position_multiturn(int, unsigned);

};

/**
 * @brief Service to exchange data between tasks OF THE SAME TILE without blocking tasks execution.
 *
 * @param Array of communication interfaces to handle n different clients
 * @param Number of supported client interfaces
 */
[[distributable]]
void memory_manager(server interface shared_memory_interface i_shared_memory[n], unsigned n);

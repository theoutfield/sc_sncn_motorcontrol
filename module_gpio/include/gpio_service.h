/**
 * @file gpio_server.h
 * @brief General Purpose IO Digital Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <xs1.h>
#include <internal_config.h>
#include <platform.h>

/**
 * @brief Lorem ipsum...
 */
interface GPIOInterface{
	/**
     * @brief Lorem ipsum...
     *
     * @param gpio_port Lorem ipsum...
     * @param input_type Lorem ipsum...
     * @param switch_type Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int config_dio_input(int gpio_port, int input_type, int switch_type);
    /**
     * @brief Lorem ipsum...
     */
    void config_dio_done();
    /**
     * @brief Lorem ipsum...
     *
     * @param gpio_port Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int read_gpio(int gpio_port);
    /**
     * @brief Lorem ipsum...
     *
     * @param gpio_port Lorem ipsum...
     * @param value Lorem ipsum...
     */
    void write_gpio(int gpio_port, int value);
};
/**
 * @fn gpio_digital_server(port p_ifm_ext_d0, port p_ifm_ext_d1, port p_ifm_ext_d2, port p_ifm_ext_d3,
 * chanend c_gpio_0, chanend c_gpio_1, chanend c_gpio_2, chanend c_gpio_3)
 * @brief Server enables configuration of digital I/Os as inputs or outputs,
 *  	   to read input ports and write to output ports
 *
 * @param gpio_ports[4] Lorem ipsum...
 * @param i_gpio[2] Lorem ipsum...
 *
 */
void gpio_service(port gpio_ports[4], interface GPIOInterface server i_gpio[2]);

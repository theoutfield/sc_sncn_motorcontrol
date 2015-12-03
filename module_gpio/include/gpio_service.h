/**
 * @file gpio_server.h
 * @brief General Purpose IO Digital Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <xs1.h>
#include <internal_config.h>
#include <platform.h>

interface GPIOInterface{
    int config_dio_input(int gpio_port, int input_type, int switch_type);
    void config_dio_done();
    int read_gpio(int gpio_port);
    void write_gpio(int gpio_port, int value);
};
/**
 * @fn gpio_digital_server(port p_ifm_ext_d0, port p_ifm_ext_d1, port p_ifm_ext_d2, port p_ifm_ext_d3,
 * chanend c_gpio_0, chanend c_gpio_1, chanend c_gpio_2, chanend c_gpio_3)
 * @brief Server enables configuration of digital I/Os as inputs or outputs,
 *  	   to read input ports and write to output ports
 * @param p_ifm_ext_d0 Digital I/O port EXT_D0
 * @param p_ifm_ext_d1 Digital I/O port EXT_D1
 * @param p_ifm_ext_d2 Digital I/O port EXT_D2
 * @param p_ifm_ext_d3 Digital I/O port EXT_D3
 * @param c_gpio_0 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D0
 * @param c_gpio_1 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D1
 * @param c_gpio_2 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D2
 * @param c_gpio_3 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D3
 *
 */
void gpio_service(port gpio_ports[4], interface GPIOInterface server i_gpio[2]);

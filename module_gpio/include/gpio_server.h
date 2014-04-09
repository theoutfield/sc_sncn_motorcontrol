
/**
 * \file gpio_server.h
 * \brief General Purpose IO Digital Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#include <xs1.h>
#include <internal_config.h>
#include <platform.h>



/**
 *	\fn gpio_digital_server(port p_ifm_ext_d0, port p_ifm_ext_d1, port p_ifm_ext_d2, port p_ifm_ext_d3,
 *	chanend c_gpio_0, chanend c_gpio_1, chanend c_gpio_2, chanend c_gpio_3)
 *  \brief Server enables configuration of digital I/Os as inputs or outputs,
 *  	   to read input ports and write to output ports
 *  \param p_ifm_ext_d0 Digital I/O port EXT_D0
 *  \param p_ifm_ext_d1 Digital I/O port EXT_D1
 *  \param p_ifm_ext_d2 Digital I/O port EXT_D2
 *  \param p_ifm_ext_d3 Digital I/O port EXT_D3
 *  \param c_gpio_0 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D0
 * 	\param c_gpio_1 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D1
 *  \param c_gpio_2 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D2
 *  \param c_gpio_3 channel used for configuration of any digital ports and send commands
 *  	   to read/write to port EXT_D3
 *
 */
void gpio_digital_server(port p_ifm_ext_d[], chanend c_gpio_0, chanend c_gpio_1);

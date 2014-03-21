#include <gpio_client.h>
#include <xscope.h>

/**
 * 	\fn config_gpio(chanend c_gpio, int port_number, int switch_type)
 *  \brief Client enables configuration of digital I/Os as inputs or outputs,
 *  	   to read input ports and write to output ports.
 *  	   By default all IO ports act as outputs if not configured as inputs.
 *
 *  \param c_gpio channel used to configure any digital ports /and send commands  to read/write to port specified
 *  \param port_number selects a port to be configured: 0 - EXT_D0, 1 - EXT_D1, 2 - EXT_D2, 3 - EXT_D3, 4 - all EXT_D0, EXT_D1, EXT_D2, EXT_D3
 *  \param switch_type specify the input
 *
 *  \return 1 - success
 *
 */
int config_gpio_digital_input(chanend c_gpio, int port_number, int input_type, int switch_type)
{
	int config_state = 1;
	if(input_type != SWITCH_INPUT_TYPE && input_type != GP_INPUT_TYPE)
		config_state = 0;
	if(port_number > 4)
		config_state = 0;

	c_gpio <: CONFIG_DIO_INPUT;
	c_gpio <: port_number;
	c_gpio <: input_type;
	c_gpio <: switch_type;

	return config_state;
}

/**
 * 	\fn end_config_gpio(chanend c_gpio)
 *  \brief Disables further configuration of digital I/Os as inputs or outputs.
 *
 *  \param c_gpio channel used to configure any digital ports
 *  \return 1 - success
 *
 */
int end_config_gpio(chanend c_gpio)
{
	c_gpio <: CONFIG_DIO_DONE;
	return 1;
}

/**
 * 	\fn config_gpio(chanend c_gpio, int port_number, int switch_type)
 *  \brief Client enables configuration of digital I/Os as inputs or outputs,
 *  	   to read input ports and write to output ports.
 *  	   By default all IO ports act as outputs if not configured as inputs.
 *
 *  \param c_gpio channel used to configure any digital ports /and send commands  to read/write to port EXT_D0
 *  	    0 - EXT_D0
 *  \param port_number selects a port to be configured
 *  \param switch_type specify the input
 *
 *  \return 1 - success
 *
 */
int read_gpio_digital_input(chanend c_gpio, int port_number)
{
	int port_value;
	c_gpio <: port_number;
	c_gpio <: GPIO_INPUT;
	c_gpio :> port_value;
	return port_value;
}

void write_gpio_digital_output(chanend c_gpio, int port_number, int port_value)//bool
{
	c_gpio <: port_number;
	c_gpio <: GPIO_OUTPUT;
	c_gpio <: port_value;
}

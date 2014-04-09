
/**
 * \file gpio_client.xc
 * \brief General Purpose IO Digital Client Functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#include <gpio_client.h>
#include <xscope.h>

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

int end_config_gpio(chanend c_gpio)
{
	c_gpio <: CONFIG_DIO_DONE;
	return 1;
}

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

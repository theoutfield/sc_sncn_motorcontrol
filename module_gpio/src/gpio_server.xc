
/**
 * \file gpio_server.xc
 * \brief General Purpose IO Digital Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#include <gpio_server.h>
#include <xscope.h>


void xscope_initialise_2()
{
	xscope_register(4, XSCOPE_CONTINUOUS, "0 port 1", XSCOPE_INT,	"n",
						XSCOPE_CONTINUOUS, "1 port 2", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "2 port 3", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "3 port 4", XSCOPE_INT, "n");

	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}
//input types

#define INPUT_PORT			1
#define OUTPUT_PORT 		2
#define NUMBER_OF_IO_PORTS	4           /**< Defines number of Digital IOs available. */


void gpio_config_handler(chanend c_gpio, port p_ifm_ext_d[], int port_function[], int active_states[], int input_types[], int &all_ports_input)
{
	int input_type;
	int switch_type;
	int port_number;
	int i;

	c_gpio :> port_number;
	c_gpio :> input_type;
	c_gpio :> switch_type;

	if(port_number < NUMBER_OF_IO_PORTS)
	{
		port_function[port_number] = INPUT_PORT;
		if(input_type == SWITCH_INPUT_TYPE)
		{
			if(switch_type == ACTIVE_HIGH)
			{
				set_port_pull_down(p_ifm_ext_d[port_number]);
				active_states[port_number] = 1;
			}
			else if(switch_type == ACTIVE_LOW)
			{
				active_states[port_number] = 0;
				/**< external pull ups required for active low digital input */
			}
			input_types[port_number] = input_type;
		}
		else if(input_type == GP_INPUT_TYPE)
		{
			input_types[port_number] = input_type;
			active_states[port_number] = 0; // invalid
		}
	}
	else if(port_number == NUMBER_OF_IO_PORTS)
	{
		all_ports_input = 1;
		if(input_type == SWITCH_INPUT_TYPE)
		{
			if(switch_type == ACTIVE_HIGH)
			{
				for(i = 0; i < NUMBER_OF_IO_PORTS; i++)
				{
					set_port_pull_down(p_ifm_ext_d[i]);
					port_function[i] = INPUT_PORT;
					input_types[i] = input_type;
					active_states[i] = 1;
				}
			}
			else if(switch_type == ACTIVE_LOW)
			{
				for(i = 0; i < NUMBER_OF_IO_PORTS; i++)
				{
					port_function[i] = INPUT_PORT;
					input_types[i] = input_type;
					active_states[i] = 0;
				}
				/**< external pull ups required for active low digital input */
			}
		}
		else if(input_type == GP_INPUT_TYPE)
		{
			for(i = 0; i < NUMBER_OF_IO_PORTS; i++)
			{
				port_function[i] = INPUT_PORT;
				input_types[i] = input_type;
				active_states[i] = 0; // invalid
			}
		}
	}
}

void gpio_client_handler(chanend c_gpio, port p_ifm_ext_d[],\
		int port_number, int type, int input_types[], int port_switch_states[], int input, int output, int i)
{
	if(port_number < NUMBER_OF_IO_PORTS)
	{
		c_gpio :> type;
		if(type == GPIO_INPUT)
		{
			if(input_types[port_number] == SWITCH_INPUT_TYPE)
			{
				c_gpio <: port_switch_states[port_number];
			}
			else
			{
				p_ifm_ext_d[port_number] :> input;
				c_gpio <: input;
			}
		}
		else if(type == GPIO_OUTPUT)
		{
			c_gpio :> output;
			p_ifm_ext_d[port_number] <: output;
		}
	}
	else if(port_number == NUMBER_OF_IO_PORTS)
	{
		c_gpio :> type;
		if(type == GPIO_INPUT)
		{
			for(i = 0 ; i < NUMBER_OF_IO_PORTS; i++)
			{
				if(input_types[i] == SWITCH_INPUT_TYPE)
				{
					c_gpio <: port_switch_states[i];
				}
				else
				{
					if(i < NUMBER_OF_IO_PORTS)
					{
						p_ifm_ext_d[i] :> input;

						c_gpio <: input;
					}
				}
			}
		}
		else if(type == GPIO_OUTPUT)
		{
			c_gpio :> output;
			for(i = 0 ; i < NUMBER_OF_IO_PORTS; i++)
			{
				p_ifm_ext_d[i] <: output;
			}
		}
	}
}

void gpio_digital_server(port p_ifm_ext_d[], chanend c_gpio_0, chanend c_gpio_1)
{
	int port_state[NUMBER_OF_IO_PORTS];
	int port_function[NUMBER_OF_IO_PORTS];
	int port_switch_states[NUMBER_OF_IO_PORTS];
	int active_states[NUMBER_OF_IO_PORTS];
	int input_types[NUMBER_OF_IO_PORTS];
	int input_type;
	int switch_type;
	int all_ports_input = 0;
	timer t;
	unsigned int time;
	int command;
	int type;
	int new_pin;
	int input;
	int output;
	int config_state = 0;
	int i;
	//xscope_initialise_2();

	for(i = 0; i < NUMBER_OF_IO_PORTS; i++)
	{
		port_state[i] = 0;
		port_switch_states[i] = 0;
		port_function[i] = OUTPUT_PORT; 	/**< By default all IO ports are configured as outputs. */
	}

	t:>time;
	while(1)
	{
		select
		{
			case c_gpio_0 :> command:
				switch(command)
				{
					case CONFIG_DIO_INPUT:
						gpio_config_handler(c_gpio_0, p_ifm_ext_d, port_function, active_states, input_types, all_ports_input);
						break;

					case CONFIG_DIO_DONE:
						config_state = 1;
						break;

					default:
						break;
				}
				break;

			case c_gpio_1 :> command:
				switch(command)
				{
					case CONFIG_DIO_INPUT:
						gpio_config_handler(c_gpio_1, p_ifm_ext_d, port_function, active_states, input_types, all_ports_input);
						break;

					case CONFIG_DIO_DONE:
						config_state = 1;
						break;

					default:
						break;
				}
				break;
		}
		if(config_state == 1)
			break;
	}

	for(i = 0; i < NUMBER_OF_IO_PORTS; i++)
	{
		if(port_function[i] == INPUT_PORT)
		{
			if(input_types[i] == SWITCH_INPUT_TYPE)
			{
				p_ifm_ext_d[i] :> port_state[i];
				if(port_state[i] == active_states[i])
				{
					port_switch_states[i] = 1;
				}
				else
					port_switch_states[i] = 0;
			}
		}
	}

	while(1)
	{

	//	for(i = 0 ; i< NUMBER_OF_IO_PORTS; i++)
	//		xscope_probe_data(i, port_switch_states[i]);

#pragma ordered
		select
		{

			case c_gpio_0 :> command:
				gpio_client_handler(c_gpio_0, p_ifm_ext_d,\
						command, type, input_types, port_switch_states, input, output, i);
				break;

			case c_gpio_1 :> command:
				gpio_client_handler(c_gpio_1, p_ifm_ext_d,\
						command, type, input_types, port_switch_states, input, output, i);
				break;

			case p_ifm_ext_d[0] when pinsneq(port_state[0]) :> port_state[0]:
				p_ifm_ext_d[0] :> new_pin;
				if(new_pin ==  port_state[0])
				{
					p_ifm_ext_d[0] :> new_pin;
					if(new_pin == port_state[0])
					{
						if(port_state[0] == active_states[0])
						{
							port_switch_states[0] = 1;
						}
						else
							port_switch_states[0] = 0;
					}
				}
				break;

			case p_ifm_ext_d[1] when pinsneq(port_state[1]) :> port_state[1]:
					p_ifm_ext_d[1] :> new_pin;
					if(new_pin ==  port_state[1])
					{
						p_ifm_ext_d[1] :> new_pin;
						if(new_pin == port_state[1])
						{
							if(port_state[1] == active_states[1])
							{
								port_switch_states[1] = 1;
							}
							else
								port_switch_states[1] = 0;
						}
					}
					break;

			case p_ifm_ext_d[2] when pinsneq(port_state[2]) :> port_state[2]:
					p_ifm_ext_d[2] :> new_pin;
					if(new_pin ==  port_state[2])
					{
						p_ifm_ext_d[2] :> new_pin;
						if(new_pin == port_state[2])
						{
							if(port_state[2] == active_states[2])
							{
								port_switch_states[2] = 1;
							}
							else
								port_switch_states[2] = 0;
						}
					}
					break;

			case p_ifm_ext_d[3] when pinsneq(port_state[3]) :> port_state[3]:
					p_ifm_ext_d[3] :> new_pin;
					if(new_pin ==  port_state[3])
					{
						p_ifm_ext_d[3] :> new_pin;
						if(new_pin == port_state[3])
						{
							if(port_state[3] == active_states[3])
							{
								port_switch_states[3] = 1;
							}
							else
								port_switch_states[3] = 0;
						}
					}
					break;

		}
	}
}


/**
 * \file test_gpio_digital.xc
 * \brief Test illustrates configuration and usage of GPIO digital ports. Two ports are configured
 *  as input switches and the remaining two ports are configured as digital out ports
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <refclk.h>
#include <internal_config.h>
#include <gpio_server.h>
#include <gpio_client.h>

#define TILE_ONE 0
#define IFM_TILE 3


/*
 * Test GPIO Client Loop
 */
void gpio_test(chanend c_gpio_p1)
{
	int port_0_value;
	int port_1_value;
	int port_2_value = 0;
	int port_3_value = 0;


	/**< Configure gpio digital port 0 as Switch Input and active high */
	config_gpio_digital_input(c_gpio_p1, 0, SWITCH_INPUT_TYPE, ACTIVE_HIGH);

	/**< Configure gpio digital port 1 as Switch Input and active high,
	 * by default other gpio digital port configured as digital outputs */
	config_gpio_digital_input(c_gpio_p1, 1, SWITCH_INPUT_TYPE, ACTIVE_HIGH);

	/**< End configuration of digital ports */
	end_config_gpio(c_gpio_p1);

	while(1)
	{
		/**< Read value on digital port 0 */
		port_0_value = read_gpio_digital_input(c_gpio_p1, 0);

		/**< Read value on digital port 1 */
		port_1_value = read_gpio_digital_input(c_gpio_p1, 1);


		printstr("Port 0 value: ");
		printint(port_0_value);
		printstr(" Port 1 value: ");
		printintln(port_1_value);


		/**< Write port_2_value to digital port 2 */
		write_gpio_digital_output(c_gpio_p1, 2, port_2_value);

		/**< Write port_3_value to digital port 3 */
		write_gpio_digital_output(c_gpio_p1, 3, port_3_value);
	}
}

int main(void)
{
	/* GPIO Communication channels */
	chan c_gpio_p1, c_gpio_p2;

	par
	{
		/* Test GPIO Client Loop */
		on stdcore[TILE_ONE] :
		{
			gpio_test(c_gpio_p1);
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on stdcore[IFM_TILE]:
		{
			/* GPIO Digital Server */
			gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);
		}

	}

	return 0;
}

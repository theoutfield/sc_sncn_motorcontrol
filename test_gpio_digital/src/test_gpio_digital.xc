
/**
 * \file test_gpio_digital.xc
 * \brief Test illustrates configuration and usage of GPIO digital ports. Two ports are configured
 *  as input switches and the remaining two ports are configured as digital out ports
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <refclk.h>
#include <xscope.h>
#include <internal_config.h>
#include <gpio_server.h>
#include <gpio_client.h>

//#define ENABLE_xscope
#define COM_CORE 0
#define IFM_CORE 3

void xscope_initialise_1()
{
	xscope_register(2, XSCOPE_CONTINUOUS, "0 port_0", XSCOPE_INT,	"n",
						XSCOPE_CONTINUOUS, "1 port_1", XSCOPE_INT, "n");

	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}

/*
 * Test GPIO Client Loop
 */
void gpio_test(chanend c_gpio_p1)
{
	int port_0_value;
	int port_1_value;
	int port_2_value = 0;
	int port_3_value = 0;

	#ifdef ENABLE_xscope
		xscope_initialise_1();
	#endif

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

		#ifdef ENABLE_xscope
			xscope_probe_data(0, port_0_value);
			xscope_probe_data(1, port_1_value);
		#else
			printstr("Port 0 value: ");
			printint(port_0_value);
			printstr(" Port 1 value: ");
			printintln(port_1_value);
		#endif

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
		on stdcore[1] :
		{
			gpio_test(c_gpio_p1);
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			/* GPIO Digital Server */
			gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);
		}

	}

	return 0;
}

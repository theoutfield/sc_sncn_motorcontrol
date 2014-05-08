
/**
 * \file a4935.xc
 * \brief Driver file for motor
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#include <xs1.h>
#include <platform.h>
#include <a4935.h>
#include <print.h>

void a4935_init(int configuration, out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn)
{
	timer timer1;
	unsigned int time1;

	configuration |= A4935_BIT_ESF; // add enable_stop_on_fault to config bits

	// set config pins and trigger reset
	p_ifm_esf_rstn_pwml_pwmh <: configuration;

	timer1 :> time1;
	//timer1 when timerafter(time1 + A4935_AFTER_RESET_DELAY) :> time1;
	timer1 when timerafter(time1 + (4 * USEC_FAST/*TICKS_US*/)) :> time1; // hold reset for at least 3.5us

	/* enable pull-ups for ff1 and ff2, as these are open-drain outputs
	 and configure as inputs as long as we are just waiting for an
	 error to occur */
	//configure_in_port_no_ready(p_ff1);
	//configure_in_port_no_ready(p_ff2);

	// release reset
	p_ifm_esf_rstn_pwml_pwmh <: ( A4935_BIT_RSTN | configuration );

	// pause before enabling FETs after reset
	timer1 when timerafter(time1 + A4935_AFTER_RESET_DELAY) :> time1;

	// enable FETs redundant with watchdog
	p_ifm_coastn <: 1;
}


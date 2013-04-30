/**
 * \file a4935.xc
 *
 *	Driver file for motor
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/


#include <xs1.h>
#include "platform.h"
#include "ioports.h"
#include "a4935.h"

static unsigned cur_state_ff1, cur_state_ff2;

//void a4935_init(port out p_esf_rstn_pwml_pwmh, port out p_coastn, unsigned configuration)
void a4935_init(unsigned configuration)
{
  timer timer1;
  unsigned time1;

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

  // enable FETs
  p_ifm_coastn <: 1;

}



//select a4935_check_fault_select(port p_ff1, port p_ff2)
select a4935_check_fault_select(void)
{
  case p_ifm_ff1 when pinsneq(cur_state_ff1) :> cur_state_ff1:
   // printstr("FF1, FF2 = ");
   // printuint(cur_state_ff1);
   // printstr(", ");
   // printuintln(cur_state_ff2);
    break;

  case p_ifm_ff2 when pinsneq(cur_state_ff2) :> cur_state_ff2:
   // printstr("FF1, FF2 = ");
   // printuint(cur_state_ff1);
   // printstr(", ");
   // printuintln(cur_state_ff2);
    break;
}

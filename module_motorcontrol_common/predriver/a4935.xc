
/**
 * @file a4935.xc
 * @brief Driver file for motor
 * @author Martin Schwarz <mschwarz@synapticon.com>
 */

#include "a4935.h"

#include <stdint.h>
#include <xs1.h>
#include <platform.h>

#include <refclk.h>

/* Reset timing definitions */
#define A4935_HOLD_RESET_DELAY (4 * USEC_FAST) /* at least 3.5us (acc. to datasheet) */
#define A4935_AFTER_RESET_DELAY (200 * MSEC_FAST)


void a4935_initialize(out port p_esf_rstn_pwml_pwmh, out port p_coastn, uint8_t configuration)
{
    timer t;
    unsigned ts;

    // set config pins and trigger reset
    p_esf_rstn_pwml_pwmh <: (unsigned) (configuration & ~A4935_BIT_RSTN);

    // hold reset for at least 3.5us
    t :> ts;
    t when timerafter(ts + A4935_HOLD_RESET_DELAY) :> ts;

    // release reset
    p_esf_rstn_pwml_pwmh <: (unsigned) ( configuration | A4935_BIT_RSTN );

    // pause before enabling FETs after reset
    t when timerafter(ts + A4935_AFTER_RESET_DELAY) :> void;

    // enable gate drive outputs
    p_coastn <: 1;
}

/*
  Fault Flag Truth Table
  FF1 FF2 | Description
    0 0   | No fault
    0 1   | Short to GND/Supply or shorted load
    1 0   | Overtemperature
    1 1   | VDD/VREG/Bootstrap undervoltage
  in case of an undervoltage fault, outputs will be disabled independent of ESF configuration
 */
select on_a4935_fault(port p_ff1, port p_ff2, uint8_t &flags)
{
    case p_ff1 when pinsneq(0) :> void:
        p_ff2 :> flags;         /* read ff2 state (bit0) */
        flags |= 0b10;          /* set ff1 bit (we know it's 1 - this select case was
                                   triggered by it) */
        break;

    case p_ff2 when pinsneq(0) :> void:
        p_ff1 :> flags;
        flags <<= 1;
        flags |= 0b01;
        break;
}

uint16_t a4935_read_fault_register(port p_ff1, port p_ff2)
{
    /*
      generate clk on ff2, read fault register (10 bits) on ff1
      ff2 clock high time (min) = 500ns
      ff2 clock low time (min)  = 500ns
      clock low to data valid delay (max) = 100ns
    */

    const unsigned t_clock_high = 1 * USEC_FAST;
    const unsigned t_clock_low = 1 * USEC_FAST;
    const int num_bits = 10;        /* size of the fault register */

    timer t;
    unsigned ts;
    uint16_t fault_reg = 0;

    t :> ts;

    for (int i=0; i<num_bits; i++) {
        p_ff2 <: 0;             /* generate falling clk edge */
        t when timerafter(ts + t_clock_low) :> ts;
        p_ff2 <: 1;             /* generate rising clk edge */

        p_ff1 :> >> fault_reg;  /* shift in 1 bit */

        t when timerafter (ts + t_clock_high) :> ts;
    }

    /* switch port ff2 to input mode */
    p_ff2 :> ts;

    /* shift bits into correct position since we've been shifting in from the left */
    fault_reg >>= 16 - num_bits;

    return (uint16_t) (fault_reg);
}


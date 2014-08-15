
/**
 * @file a4935.h
 * @brief Driver header file for motor
 * @author Martin Schwarz <mschwarz@synapticon.com>
*/

#pragma once

#include <stdint.h>
#include <refclk.h>

/* Pin mapping of 4-bit A4935 config port
   ESF        - Enable Stop on Fault
   RST_N      - Reset (active-low)
   PWML, PWMH - PWM control signals (refer to A4935 datasheet) */
#define A4935_BIT_ESF  0x08
#define A4935_BIT_RSTN 0x04
#define A4935_BIT_PWML 0x02
#define A4935_BIT_PWMH 0x01

// Fault Flags (FF2 = LSB)
#define A4935_FF_NO_FAULT           0x00
#define A4935_FF_SHORT_FAULT        0x01
#define A4935_FF_OVERTEMP_FAULT     0x02
#define A4935_FF_UNDERVOLTAGE_FAULT 0x03

// Fault Register Bit Mapping
#define A4935_FAULT_REGISTER_BIT_AH 0x0001 /* Vds exceeded on A phase, high-side */
#define A4935_FAULT_REGISTER_BIT_BH 0x0002 /* Vds exceeded on B phase, high-side */
#define A4935_FAULT_REGISTER_BIT_CH 0x0004 /* Vds exceeded on C phase, high-side */
#define A4935_FAULT_REGISTER_BIT_AL 0x0008 /* Vds exceeded on A phase, low-side */
#define A4935_FAULT_REGISTER_BIT_BL 0x0010 /* Vds exceeded on B phase, low-side */
#define A4935_FAULT_REGISTER_BIT_CL 0x0020 /* Vds exceeded on C phase, low-side */
#define A4935_FAULT_REGISTER_BIT_VR 0x0040 /* Undervoltage on VREG */
#define A4935_FAULT_REGISTER_BIT_VA 0x0080 /* Bootstrap undervoltage on phase A */
#define A4935_FAULT_REGISTER_BIT_VB 0x0100 /* Bootstrap undervoltage on phase B */
#define A4935_FAULT_REGISTER_BIT_VC 0x0200 /* Bootstrap undervoltage on phase C */


void a4935_initialize(out port p_esf_rstn_pwml_pwmh, out port p_coastn, uint8_t configuration);

select on_a4935_fault(port p_ff1, port p_ff2, uint8_t &flags);

uint16_t a4935_read_fault_register(port p_ff1, port p_ff2);

//void a4935_clear_fault_flags(xx);

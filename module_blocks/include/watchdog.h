/**
 * @file watchdog.h
 * @brief Watchdog Implementation
 * @author Victor de la Cruz <vdelacruz@synapticon.com>
 */

#pragma once

#define WD_CMD_EN_MOTOR  1
#define WD_CMD_DIS_MOTOR 2
#define WD_CMD_TICK      3
#define WD_CMD_START     4

/** @brief Run the watchdog timer server
 *
 * The watchdog timer needs a constant stream of pulses to prevent it
 * from shutting down the motor.  This is a thread server which implements
 * the watchdog timer output.
 *
 * The watchdog control has two differents ports attached to
 * the watchdog circuitry. The enable signal must be the LSB
 * bit in a 4-bit port. The tick control must be a 1-bit port.
 *
 * @param c_watchdog the control channel for controlling the watchdog
 * @param p_wd_tick control for the tick of the watchdog
 * @param p_shared_leds_wden control port for the watchdog device
 */
void run_watchdog(chanend c_watchdog, out port p_wd_tick, out port p_shared_leds_wden);


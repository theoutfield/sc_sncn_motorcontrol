/**
 * @file watchdog.h
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

typedef struct {
    out port p_enable;
    out port ?p_tick;
} WatchdogPorts;

interface WatchdogInterface{
    void start(void);
    void enable_motors(void);
    void disable_motors(void);
};

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
void run_watchdog(interface WatchdogInterface server watchdog_interface, WatchdogPorts &watchdog_ports);


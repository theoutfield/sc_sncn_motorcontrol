/**
 * @file watchdog.h
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**
* @brief Lorem ipsum...
*/
typedef struct {
    out port p_enable;
    out port ?p_tick;
} WatchdogPorts;

/**
* @brief Lorem ipsum...
*/
interface WatchdogInterface{
	/**
     * @brief Lorem ipsum...
     */
    void start(void);
    /**
     * @brief Lorem ipsum...
     */
    void stop(void);
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
 * @param watchdog_ports Lorem ipsum...
 * @param watchdog_interface[2] Lorem ipsum...
 */
[[combinable]]
void watchdog_service( WatchdogPorts &watchdog_ports, interface WatchdogInterface server watchdog_interface[2]);


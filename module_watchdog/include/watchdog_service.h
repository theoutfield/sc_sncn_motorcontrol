/**
 * @file watchdog_service.h
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motor_control_interfaces.h>


/**
 * Structure type for Watchdog Service ports
 */
typedef struct {
    out port p_enable; /**< 4-bit Port for Watchdog basic management. */
    out port ?p_tick; /**< [Nullable] Port for the periodic tick signal (if applicable in your SOMANET device). */
} WatchdogPorts;

/**
 * @brief Service to manage the watchdog chip within your IFM SOMANET device.
 *
 * @param watchdog_ports Ports structure defining where to access the watchdog chip.
 * @param i_watchdog Array of communication interfaces to handle up to 2 different clients.
 */
[[combinable]]
void watchdog_service( WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[2], int ifm_tile_usec);


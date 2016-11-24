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
    out port ?p_shared_enable_tick_led; /**< 4-bit Port for Watchdog basic management. */
    out port ?p_tick; /**< [Nullable] Port for the periodic tick signal (if applicable in your SOMANET device). */
    port ?p_cpld_shared;
} WatchdogPorts;

/**
 * @brief Enumeration of existing IFM modules.
 */
enum {DC100_DC300, DC500, DC1K_DC5K};

/**
 * @brief Service to manage the watchdog chip within your IFM SOMANET device.
 *
 * @param watchdog_ports Ports structure defining where to access the watchdog chip.
 * @param i_watchdog Array of communication interfaces to handle up to 2 different clients.
 */
[[combinable]]
void watchdog_service( WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[2], int ifm_tile_usec);

void blink_red(int fault, int period, WatchdogPorts &watchdog_ports, int IFM_module_type, unsigned char &output, unsigned int &times, unsigned int &delay_counter);

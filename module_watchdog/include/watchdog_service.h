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
    port ?p_diag_enable; /**< Nullable] Bidirectional port for Diagnostic and enable */
    out port ?p_tick; /**< [Nullable] Port for the periodic tick signal (if applicable in your SOMANET device). */
    port ?p_cpld_shared;
    port ?p_cpld_fault_monitor;
} WatchdogPorts;

/**
 * @brief Enumeration of existing IFM modules.
 */
enum {DC100_DC300, DC500, DC1K_DC5K, DC1KD1, DC30};

/**
 * @brief Service to manage the watchdog chip within your SOMANET Drive device.
 *
 * @param watchdog_ports Ports structure defining watchdog chip HW access.
 * @param i_watchdog Array of communication interfaces to handle up to 2 different clients.
 */
[[combinable]]
void watchdog_service( WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[2], int tile_usec);

/**
 * @brief Function to blink red LED on your SOMANET Drive devices to signal different types of faults.
 *
 * @param fault Fault ID, i.e., number of times to blink followed by a pause.
 * @param period Red LED on/off period
 * @param watchdog_ports Ports structure defining watchdog chip HW access.
 * @param IFM_module_type Type of the IFM board used to properly handle the ports.
 * @param output Value to be written to the shared LED port
 * @param output_cpld Value to be written to the shared LED port for cpld boards
 * @param times Internal blinking counter
 * @param delay_counter External counter incrementing every WD clock cycle
 */
void blink_red(int fault, int period, WatchdogPorts &watchdog_ports, int IFM_module_type, unsigned char &output, unsigned &output_cpld, unsigned int &times, unsigned int &delay_counter);

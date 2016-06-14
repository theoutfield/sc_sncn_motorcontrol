/**
 * @file watchdog_service.h
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

typedef struct wd_ports {
    out port ? p_wd_tick;
    out port ? p_shared_leds_wden;
    out port ? p_wd_enable;
    unsigned shared_pin_wd_enable;
    unsigned shared_pin_wd_tick;
    unsigned keep_alive_ticks;
} WatchdogPorts;


typedef enum {
        WD_STATE_OFF,
        WD_STATE_EN,
}WatchdogState;


/**
 * @brief Interface type to communicate with the Watchdog Service.
 */
interface WatchdogInterface{

	/**
     * @brief Initialize and starts ticking the watchdog.
     */
    void start(void);

    /**
     * @brief Stops ticking the watchdog. Therefore, any output through the phases is disabled.
     */
    void stop(void);


    void enableKeepAliveTimeout();

    void disableKeepAliveTimeout();


    WatchdogState getState();


    /**
     * @brief External keepalive from other processes, resets timeout
     */
    void keepalive();
};

/**
 * @brief Service to manage the watchdog chip within your IFM SOMANET device.
 *
 * @param watchdog_ports Ports structure defining where to access the watchdog chip.
 * @param i_watchdog Array of communication interfaces to handle up to 2 different clients.
 */
[[combinable]]
void watchdog_service( WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[4]);


/**
 * @file gpio_server.h
 * @brief General Purpose IO Digital Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

/**
 * @brief Type definition for port configuration.
 */
typedef enum { ACTIVE_HIGH=1, ACTIVE_LOW }SwitchType;

enum { HOMING_NEGATIVE_SWITCH=1, HOMING_POSITIVE_SWITCH };

#ifdef __XC__

#include <xs1.h>
#include <mc_internal_constants.h>
#include <platform.h>

/**
 * @brief Interface type to communicate with the GPIO Service.
 */
interface GPIOInterface{

	/**
     * @brief   Set the configuration of the GPIOs.
     *          By default all GPIOs are configured as outputs.
     *
     * @param gpio_port Number of the pin to be configured [0:3].
     * @param input_type Type of input [GP_INPUT_TYPE, SWITCH_INPUT_TYPE].
     * @param switch_type [ACTIVE_HIGH, ACTIVE_LOW].
     *
     * @return 0 - Error
     *         1 - Success
     */
    int config_dio_input(int gpio_port, int input_type, SwitchType switch_type);

    /**
     * @brief Disables further configuration of any GPIO.
     *        Required before starting reading or writing values.
     */
    void config_dio_done();

    /**
     * @brief Read the current value in a GPIO pin.
     *
     * @param gpio_port Number of the pin to read [0:3].
     * @return Read value in the port [0,1].
     */
    int read_gpio(int gpio_port);

    /**
     * @brief Set a value in certain GPIO.
     *
     * @param gpio_port Number of the pin to write to [0:3].
     * @param value Output value for the pin [0,1].
     */
    void write_gpio(int gpio_port, int value);
};
/**
 * @brief Service to manage the digital I/Os in your SOMANET device.
 *
 * @param gpio_ports[4] Array of ports to be managed.
 * @param i_gpio[2] Array of communication interfaces to handle one client.
 *
 */
void gpio_service(port gpio_ports[4], interface GPIOInterface server i_gpio[1]);

#endif

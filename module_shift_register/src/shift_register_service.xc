/*
 * shift_register_service.xc
 *
 *  Created on: 14.11.2017
 *      Author: hstroetgen
 */
#include <xclib.h>
#include <xs1.h>
#include <stdint.h>

#define P_DATA  0
#define P_SCK   1
#define P_RCK   2
#define P_RESET 3

#define MODULE_STATUS_G 0
#define MODULE_STATUS_R 1
#define COM_STATUS_G    2
#define COM_STATUS_R    3
#define FAULT_DETECT_R  5
#define MOTOR_VOLTAGE_G 6

void set_output(interface GPIOInterface client i_gpio, uint8_t data[], uint8_t length)
{
    uint8_t i = 0;

    for (i = 0; i < length; i++)
    {
        i_gpio.write_gpio(P_DATA, data[i]);

        i_gpio.write_gpio(P_SCK, 0); // check timing
        i_gpio.write_gpio(P_SCK, 1);
        i_gpio.write_gpio(P_SCK, 0);
    }

    i_gpio.write_gpio(P_RCK, 0); // check timing
    i_gpio.write_gpio(P_RCK, 1);
    i_gpio.write_gpio(P_RCK, 0);
}

void shift_register_service(interface GPIOInterface client i_gpio, interface i_schunk_led server i_led[n], unsigned n)
{
    static uint8_t led_data[8] = {0};
    static uint8_t motor_voltage_status = 0;
    static uint8_t com_status           = 0;
    static uint8_t module_status        = 0;
    static uint8_t fault                = 0;


    while (1)
    {
        select {
            case i_led[int j].error(uint8_t status):
                    break;

            case i_led[int j].voltage_status(uint8_t status):
                    break;

            case i_led[int j].com_status(uint8_t status):
                    break;

            case i_led[int j].module_status(uint8_t status):
                    break;
        }

    }

//    while (1)
//    {
//        if (motor_voltage_status) {
//                led_data[MOTOR_VOLTAGE_G] = 1;
//           } else {
//                led_data[MOTOR_VOLTAGE_G] = 0;
//           }
//
//           if (com_status) {
//                led_data[COM_STATUS_G] = 1;
//                led_data[COM_STATUS_R] = 0;
//           } else {
//                led_data[COM_STATUS_G] = 0;
//                led_data[COM_STATUS_R] = 1;
//           }
//
//           if (module_status) {
//                led_data[MODULE_STATUS_G] = 1;
//                led_data[MODULE_STATUS_R] = 0;
//           } else {
//                led_data[MODULE_STATUS_G] = 0;
//                led_data[MODULE_STATUS_R] = 1;
//           }
//
//           if (fault) {
//                led_data[FAULT_DETECT_R] = 1;
//           } else {
//                led_data[FAULT_DETECT_R] = 0;
//           }
//
//
//    }
}

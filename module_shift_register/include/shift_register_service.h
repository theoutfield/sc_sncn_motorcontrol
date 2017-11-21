/*
 * shift_register_service.h
 *
 *  Created on: 14.11.2017
 *      Author: hstroetgen
 */


#ifndef SHIFT_REGISTER_SERVICE_H_
#define SHIFT_REGISTER_SERVICE_H_

interface i_schunk_led
{
    void error(uint8_t status);

    void voltage_status(uint8_t status);

    void com_status(uint8_t status);

    void module_status(uint8_t status);
};


#endif /* SHIFT_REGISTER_SERVICE_H_ */

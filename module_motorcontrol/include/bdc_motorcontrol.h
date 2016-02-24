/**
 * @file
 * @brief Brushed Motor Drive Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>

[[combinable]]
void bdc_loop(chanend c_pwm_ctrl,
               interface WatchdogInterface client i_watchdog,
               interface MotorcontrolInterface server i_commutation[4],
               FetDriverPorts &fet_driver_ports,
               MotorcontrolConfig &motorcontrol_config);

.. _module_pwm:

=====================
PWM Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service (pwm_service_general) to generate center-aligned Pulse-Width modulation(PWM) signals for both high-side and low-side FETs of your IFM module. PWM module can be used to cycle on-and-off a digital signal in order to control a load which requires electrical power. As shown in figure 1, the period and hence the frequency of a pwm signal is always fixed (the default value is 12 kHz). Only the on-time of PWM pulses can be changed. The on-time (for each inverter output can be adjusted by sending the corresponding pwm_value to pwm_service_general through an interface. By this technique, it is possible to modulate a given reference voltage.

The PWM Service should always run over an **IFM Tile** so it can access the ports of your SOMANET IFM device.

If PWM signals are used to drive an IFM module, the watchdog service should also be running to activate the IFM ports.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_pwm>`_

.. image:: images/pwm-concept.png
   :width: 80%


How to use
==========

.. important:: It is assumed that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_advanced_torquecontrol_lib module_board-support module_misc module_pwm module_watchdog

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the PWM Service header **pwm_server.h** in your app.

3. Include the WATCHDOG Service header **watchdog_service.h** in your app.

4. Define the required pwm ports in the board-support-package of your IFM module. By default, these ports are defined for phase A, B and C of the IFM module.

5. Define the required watchdog ports in the board-support-package of your IFM module. By default, these ports are defined in the board-support-package of each IFM module. 

6. Inside your main function, instantiate the interfaces array for the Service-Clients communication (both watchdog service and pwm service).

7. At your IFM tile, instantiate the Services (both PWM service and watchdog service)

8. At whichever other core, now you can perform calls to the PWM Service through the interfaces connected to it.

    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        #include <pwm_server.h> // 2
        #include <watchdog_service.h> // 3

        PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL; // 4
        WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS; // 5

        int main(void)
        {
            interface update_pwm_general i_update_pwm; // 6
            interface WatchdogInterface i_watchdog[2]; // 6

            par
            {
                {
                    delay_milliseconds(1000);
                    send_pwm_values(i_update_pwm); // 8
                }

                /* PWM Service */
                {
                    pwm_config_general(pwm_ports);

                    delay_milliseconds(500);
                    pwm_service_general(pwm_ports, i_update_pwm); // 7
                }

                /* Watchdog Service */
                {
                    delay_milliseconds(200);
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC); // 7
                }
            }

            return 0;
        }

API
===

Definitions
-----------

.. doxygendefine:: BISS_SENSOR

Types
-----

.. doxygenstruct:: BISSConfig
.. doxygenstruct:: BISSPorts

Service
--------

.. doxygenfunction:: biss_service

Interface
---------

.. doxygeninterface:: BISSInterface

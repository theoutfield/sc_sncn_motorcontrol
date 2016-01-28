.. _module_watchdog:
=======================
SOMANET Watchdog Module 
=======================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will manage the Watchdog on your SOMANET device.
Up to 2 clients could control the Service through an interface.

When running the Watchdog Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The Watchdog Service should always run over an **IFM tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_watchdog>`_

How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
.. seealso:: You might find useful the :ref:`SOMANET Watchdog Driver Demo <watchdog_driver_demo>` example app, which illustrates the use of this module. 

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_watchdog module_pwm_symmetrical module_adc module_ctrl_loops module_hall module_misc module_motorcontrol module_profile module_qei module_board-support

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the Watchdog Service header **watchdog_service.h** in your app. 

3. Instanciate the ports where the Service will be accessing the ports of the watchdog chip. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instanciate the Service.

6. At whichever other core, now you can perform calls to the Watchdog Service through the interfaces connected to it.

    .. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <watchdog_service.h> // 2

        WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS; // 3

        int main(void) {

            interface WatchdogInterface i_watchdog[2]; // 4

            par
            {
                on tile[APP_TILE]: i_watchdog[0].start(); // 6

                on tile[IFM_TILE]: watchdog_service(wd_ports, i_watchdog); // 5
            }

            return 0;
        }


API
===

Types
-----

.. doxygenstruct:: WatchdogPorts

Service
-------

.. doxygenfunction:: watchdog_service

Interface
---------

.. doxygeninterface:: WatchdogInterface

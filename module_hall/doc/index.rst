.. _module_hall:
===========================
SOMANET Hall Sensor Module
===========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from your 
Feedback Hall Sensor. Up to 5 clients could retrieve data from the Service
through interfaces.

When running the Hall Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The Hall Service should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_hall>`_

.. image:: images/core-diagram-hall-interface.png
   :width: 50%


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
.. seealso:: You might find useful the :ref:`SOMANET Hall Effect Feedback Sensor Demo <hall_demo>`, which illustrates the use of this module. 

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_hall module_pwm_symmetrical module_adc module_ctrl_loops module_misc module_motorcontrol module_profile module_gpio module_qei module_watchdog module_board-support

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the Hall Service header **hall_service.h** in your app. 

3. Instanciate the ports where the Service will be reading the Hall Sensor feedback signals. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instanciate the Service. For that, first you will have to fill up your Service configuration.

6. At whichever other core, now you can perform calls to the Hall Service through the interfaces connected to it.

    .. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <hall_service.h> // 2

        HallPorts hall_ports = SOMANET_IFM_HALL_PORTS; // 3

        int main(void)
        {
            interface HallInterface i_hall[5]; // 4

            par
            {
                on tile[APP_TILE]: int foo = i_hall[0].get_hall_position(); // 6

                on tile[IFM_TILE]:
                {
                    HallConfig hall_config; // 5
                    hall_config.pole_pairs = 1;

                    hall_service(hall_ports, hall_config, i_hall);
                }
            }

            return 0;
        }

API
===

Definitions
------------

.. doxygendefine:: HALL_SENSOR

Types
-----

.. doxygenstruct:: HallConfig
.. doxygenstruct:: HallPorts

Service
-------

.. doxygenfunction:: hall_service

Interface
---------

.. doxygeninterface:: HallInterface

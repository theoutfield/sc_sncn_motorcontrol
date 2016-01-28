====================
SOMANET GPIO Module 
====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will configure, read and write
the general purpose IO pins on your SOMANET device. A client
could retrieve data from the Service through interfaces. This Service
is very useful for certain applications, such as Motor Control over
a communication protocol (e.g. EtherCAT).

When running the Hall Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The GPIO Service should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_gpio>`_

How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
1. First, add all the **SOMANET Motor Control Library** modules to your app Makefile.

    ::

        USED_MODULES = module_gpio module_hall module_pwm_symmetrical module_adc module_ctrl_loops module_misc module_motorcontrol module_profile module_qei module_watchdog module_board-support

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the GPIO Service header **gpio_service.h** in your app. 

3. Instanciate the ports where the Service will be accessing the GPIO pins. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instanciate the Service. 

6. At whichever other core, now you can perform calls to the GPIO Service through the interfaces connected to it.

    .. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <gpio_service.h> // 2

        port gpio_ports[4] = {  SOMANET_IFM_GPIO_D0,
                                SOMANET_IFM_GPIO_D1,
                                SOMANET_IFM_GPIO_D2,
                                SOMANET_IFM_GPIO_D3 }; // 3
        int main(void)
        {

            interface GPIOInterface i_gpio[1]; // 4
        
            par
            {
                on tile[APP_TILE]:
                {
                        i_gpio[0].config_dio_done(); // 6
                        int foo = i_gpio[0].read_gpio(0);                        
                }

                on tile[IFM_TILE]: gpio_service(gpio_ports, i_gpio); // 5
            }

            return 0;
        }    
API
===

Types
-----

.. doxygenenum:: SwitchType

Service
-------

.. doxygenfunction:: gpio_service

Interface
---------

.. doxygeninterface:: GPIOInterface

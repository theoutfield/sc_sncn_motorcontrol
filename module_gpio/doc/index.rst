====================
SOMANET GPIO Module 
====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will configure, read and write
the general purpose IO pins on your SOMANET device. Up to 2 clients
could retrieve data from the Service through interfaces. This Service
is very useful for certain applications, such as Motor Control over
a communication protocol (e.g. EtherCAT).

When running the Hall Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The GPIO Service should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

How to use
==========

.. important:: We assume that you are using **SOMANET Base** and your app includes the required **board support** files for your SOMANET device.
          
1. First, add all the **SOMANET Motor Control Library** modules to your app Makefile.

::

USED_MODULES = module_gpio module_hall module_pwm_symmetrical module_adc module_ctrl_loops module_misc module_motorcontrol module_profile module_qei module_watchdog module_board-support

.. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the Service header in your app. 

3. Instanciate the ports where the Service will be reading the Hall Sensor feedback signals. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instanciate the Service. For that, first you will have to fill up your Service configuration.

6. At whichever other core, now you can perform calls to the Hall Service through the interfaces connected to it.

API
===

Types
-----

.. doxygenstruct:: SwitchType

Service
-------

.. doxygenfunction:: gpio_service

Interface
---------

.. doxygeninterface:: GPIOInterface

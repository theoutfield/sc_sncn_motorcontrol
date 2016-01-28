==================
SOMANET ADC Module 
==================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from the ADC 
on your SOMANET device. Up to 2 clients could retrieve data from the Service
through interfaces.

When running the ADC Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The Hall Service should always run over an **IFM tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_adc>`_

How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
1. First, add all the **SOMANET Motor Control Library** modules to your app Makefile.

    ::

        USED_MODULES = module_adc module_pwm_symmetrical module_ctrl_loops module_hall module_misc module_motorcontrol module_profile module_qei module_watchdog module_board-support


    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the ADC Service header **adc_service.h** in your app. 

3. Instanciate the ports where the Service will be accessing the ADC chip signals. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instanciate the Service. 

6. At whichever other core, now you can perform calls to the ADC Service through the interfaces connected to it.

.. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <adc_service.h> // 2

        ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS; // 3

        int main(void)
        {
            interface ADCInterface i_adc[2] // 4

            par
            {
                on tile[APP_TILE]: 
                {
                        int foo, bar;       
                        {foo, bar} = i_adc[0].get_currents(); // 6
                }

                on tile[IFM_TILE]: adc_service(adc_ports, null, i_adc) // 5

            }

            return 0;
        }

API
===

Types
-----

.. doxygenstruct:: AD7949Ports
.. doxygenstruct:: AD7265Ports
.. doxygenstruct:: ADCPorts

Service
-------

.. doxygenfunction:: adc_service

Interface
---------

.. doxygeninterface:: ADCInterface

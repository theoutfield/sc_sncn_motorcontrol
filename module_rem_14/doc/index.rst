    .. _module_biss:

=====================
BiSS Encoder Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service which will read and process the data coming from your BiSS Encoder Feedback Sensor. Up to 5 clients could retrieve data from the Service through an interface.

BiSS is an open source digital interface for sensors and actuators. BiSS is hardware compatible to the industrial standard SSI (Serial Synchronous Interface). The standardization process is coordinated on biss-interface.com_.

When running the BiSS Service, the **Reference Frequency** of the tile where the Service is
allocated will be automatically changed to **250MHz**.

The BiSS Service should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_biss>`_

.. image:: images/core-diagram-biss-interface.png
   :width: 50%

.. _biss-interface.com: http://www.biss-interface.com/


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`BiSS Sensor Demo <biss_demo>`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_biss module_board-support module_misc

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the BiSS Service header **biss_service.h** in your app.

3. Instantiate the ports where the Service will be sending the BiSS clock, reading the BiSS Sensor feedback signals and the clock block to use.

4. Inside your main function, instantiate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instantiate the Service. For that, first you will have to fill up your Service configuration.

6. At whichever other core, now you can perform calls to the BiSS Service through the interfaces connected to it.

    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        #include <biss_service.h> // 2

        BiSSPorts biss_ports = SOMANET_IFM_BISS_PORTS; // 3

        int main(void)
        {
            interface BiSSInterface i_biss[5]; // 4

            par
            {
                on tile[APP_TILE]: int foo = i_biss[0].get_biss_position(); // 6

                on tile[IFM_TILE]:
                {
                    BiSSConfig biss_config; // 5
                    biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    biss_config.status_length = BISS_STATUS_LENGTH;
                    biss_config.crc_poly = BISS_CRC_POLY;
                    biss_config.pole_pairs = 2;
                    biss_config.polarity = BISS_POLARITY;
                    biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    biss_config.timeout = BISS_TIMEOUT;
                    biss_config.max_ticks = BISS_MAX_TICKS;
                    biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                    biss_service(biss_ports, biss_config, i_biss);
                }
            }

            return 0;
        }

API
===

Definitions
-----------

.. doxygendefine:: DEFAULT_SPI_CLOCK_DIV

Types
-----

.. doxygenenum:: REM_14_ABIResolution
.. doxygenenum:: REM_14_DynAngleComp
.. doxygenenum:: REM_14_Noise
.. doxygenenum:: REM_14_Hysteresis
.. doxygenestruct:: REM_14Config

Functions
--------

.. doxygenfunction:: initRotarySensorInterface
.. doxygenfunction:: initRotarySensor
.. doxygenfunction:: readZeroPosition
.. doxygenfunction:: readNumberPolePairs
.. doxygenfunction:: readSettings1
.. doxygenfunction:: readSettings2
.. doxygenfunction:: readRedundancyReg
.. doxygenfunction:: readProgrammingReg
.. doxygenfunction:: readCORDICMagnitude
.. doxygenfunction:: readRotaryDiagnosticAndAutoGainControl
.. doxygenfunction:: readRotarySensorError
.. doxygenfunction:: readRotarySensorAngleWithoutCompensation
.. doxygenfunction:: readRotarySensorAngleWithCompensation
.. doxygenfunction:: writeZeroPosition
.. doxygenfunction:: writeNumberPolePairs


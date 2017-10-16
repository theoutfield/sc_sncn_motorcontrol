.. _module_biss_encoder:

=========================
BiSS / SSI Encoder Module
=========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides functions to read data transmitted using the BiSS protocol and process the data coming from a BiSS Encoder into position data. The module is also compatible with SSI encoders.

BiSS is an open source digital interface for sensors and actuators. BiSS is hardware compatible to the industrial standard SSI (Serial Synchronous Interface). The standardization process is coordinated on biss-interface.com_.

Those functions are used in :ref:`Serial Encoder Module <module_serial_encoder>` itself used by :ref:`Position Feedback Module <module_position_feedback>` to create a service for reading a BiSS or SSI encoders.

The BiSS functions should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_biss>`_

.. _biss-interface.com: http://www.biss-interface.com/


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`BiSS Sensor Demo <app_test_biss>` and :ref:`SSI Sensor Demo <app_test_ssi>`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = configuration_parameters module_adc module_biss_encoder lib_bldc_torque_control module_board-support module_hall_sensor module_utils module_position_feedback module_pwm module_incremental_encoder module_encoder_rem_14 module_encoder_rem_16mt module_serial_encoder module_shared_memory module_spi_master module_watchdog 

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the BiSS Service header **biss_service.h** in your app.

3. Instantiate the ports for the BiSS.

     BiSS needs a clock output port, a data input port and a clock block. The read function takes pointers for parameters, so you need to pass the addresses of the ports.
     The ``biss_clock_low`` and ``biss_clock_high`` parameters need to be set. When the clock port is a 1-bit port they should be ``0`` and ``1``. 
     When the clock port is more than 1 bit and the other bits are used for external config ``biss_clock_low`` and ``biss_clock_high`` should be set accordingly.

4. Fill up the BiSS configuration structure. BiSS and SSI use the same parameters. The protocol type is selected with the ``position_feedback_config.sensor_type`` parameters. Usually SSI encoders don't support CRC or multiturn, they can be disabled by setting them to `0`.

5. At your IFM tile, You can use the functions to read BiSS data and process it into position data.
    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        // 2. Include the BiSS Service header **biss_service.h** in your app.
        #include <biss_service.h>
        
        // 3.Instantiate the ports for the BiSS.
        port ? qei_hall_port_2 = SOMANET_IFM_ENCODER_2_PORT;
        port ? gpio_port_3 = SOMANET_IFM_GPIO_D3; // 1-bit port

        int main(void)
        {
            par
            {
                on tile[IFM_TILE]:
                {
                    // 4. Fill up the BiSS configuration structure.                 
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = BISS_SENSOR; // or SSI_SENSOR for SSI
                    position_feedback_config.ifm_usec    = IF2_TILE_USEC;
                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.busy = BISS_BUSY;
                    position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                    position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;
                    position_feedback_config.biss_config.data_port_signal_type = BISS_DATA_PORT_SIGNAL_TYPE;
                    
                    // 5. Use the functions to read BiSS data and process it into position data.
                    // read BiSS data
                    int data[BISS_FRAME_BYTES]; // array of 32 bit bytes to store the data. The size needs to be enough to store all the data bits. 
                    timer t;
                    int biss_clock_low = 0;
                    int biss_clock_high = 1;
                    int error = read_biss_sensor_data(&gpio_port_3, &qei_hall_port_2, biss_clock_low, biss_clock_high, t, position_feedback_config, data);
                    // process data
                    int count, position, status;
                    { count, position, status } = biss_encoder(data, position_feedback_config);
                }
            }

            return 0;
        }

API
===

Definitions
-----------

.. doxygendefine:: BISS_FRAME_BYTES
.. doxygendefine:: BISS_DATA_PORT_BIT
.. doxygendefine:: BISS_STATUS_BITS

Types
-----

.. doxygenstruct:: PositionFeedbackConfig
.. doxygenstruct:: BISSConfig
.. doxygenenum:: SensorError
.. doxygenenum:: EncoderPortNumber
.. doxygenenun:: BISSClockPortConfig
.. doxygenstruct:: QEIHallPort
.. doxygenstruct:: HallEncSelectPort
.. doxygenstruct:: SPIPorts

Functions
--------

.. doxygenfunction:: read_biss_sensor_data
.. doxygenfunction:: biss_encoder
.. doxygenfunction:: biss_crc
.. doxygenfunction:: biss_crc_correct


.. _module_biss:

=====================
BiSS Encoder Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a functions to read data transmitted using the BiSS protocol and process the data coming from a BiSS Encoder into position data.

BiSS is an open source digital interface for sensors and actuators. BiSS is hardware compatible to the industrial standard SSI (Serial Synchronous Interface). The standardization process is coordinated on biss-interface.com_.

Those functions are used in :ref:`Serial Encoder Module <module_serial_encoder>` itself used by :ref:`Position Feedback Module <module_position_feedback>` to create a service for reading a BiSS encoder.

The BiSS functions should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_biss>`_

.. _biss-interface.com: http://www.biss-interface.com/


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`BiSS Sensor Demo <biss_demo>`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = config_motor module_adc module_biss module_bldc_torque_control_lib module_board-support module_hall module_misc module_position_feedback module_pwm module_qei module_rem_14 module_rem_16mt module_serial_encoder module_shared_memory module_spi_master module_watchdog 

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the BiSS Service header **biss_service.h** in your app.

3. Instantiate the ports for the BiSS.

     BiSS needs an clock output port, a data input port and a clock block. The ports structures are defined in ``position_feedback_service.h``.
     The clock block is taken for the SPI ports sturcture. Depending on the BiSS configuration the output clock port is taken from ``hall_enc_select_port`` or from a GPIO port and the data input port is taken from ``qei_hall_port`` ``1`` or ``2``.
     The functions take pointers for parameters, so you need to pass the addresses of the port structures. If qei_hall_ports are used the ``hall_enc_select_config`` parameter also needs to be set to configure them in differential mode.

4. Fill up the BiSS configuration structure.

5. At your IFM tile, You can use the functions to read BiSS data and process it into position data.
    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        // 2. Include the BiSS Service header **biss_service.h** in your app.
        #include <biss_service.h>
        
        // 3.Instantiate the ports for the BiSS.
        SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
        QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
        QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
        HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;
        port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;

        int main(void)
        {
            par
            {
                on tile[IFM_TILE]:
                {
                    // 4. Fill up the BiSS configuration structure.
                    BiSSConfig biss_config; 
                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.busy = BISS_BUSY;
                    position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                    position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;
                    
                    // 5. Use the functions to read BiSS data and process it into position data.
                    // read BiSS data
                    int data[BISS_FRAME_BYTES]; // 32 bit array to store the data. The size need to be enough to store all the data bits. 
                    int hall_enc_select_config = 0b0011; //to configure qei_hall_ports in differential mode
                    int error = read_biss_sensor_data(&qei_hall_port_1, &qei_hall_port_2, &hall_enc_select_port, hall_enc_select_config, &gpio_port_2, biss_config, data);
                    // process data
                    int count, position, status;
                    { count, position, status } = biss_encoder(data, biss_config);
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

.. doxygenenum:: BISS_ErrorType
.. doxygenenun:: BISSClockPortConfig
.. doxygenstruct:: BISSConfig
.. doxygenstruct:: QEIHallPort
.. doxygenstruct:: HallEncSelectPort
.. doxygenstruct:: SPIPorts

Functions
--------

.. doxygenfunction:: read_biss_sensor_data
.. doxygenfunction:: biss_encoder
.. doxygenfunction:: biss_crc
.. doxygenfunction:: biss_crc_correct


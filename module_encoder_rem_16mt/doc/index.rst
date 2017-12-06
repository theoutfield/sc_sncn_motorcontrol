.. _module_encoder_rem_16mt:

=======================
REM 16MT Encoder Module
=======================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides functions to read  data coming from a REM 16MT Encoder.

Those functions are used in :ref:`Serial Encoder Module <module_serial_encoder>` itself used by :ref:`Position Feedback Module <module_position_feedback>` to create a service for reading a REM 16MT encoder.

The functions should always run over an **IF2 Tile** so it can access the ports to
your SOMANET Drive module.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_rem_16mt>`_


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`REM 16MT Sensor Demo <app_test_rem_16mt>`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = configuration_parameters module_adc module_encoder_rem_16mt lib_bldc_torque_control module_board-support module_hall_sensor module_utils module_position_feedback module_pwm module_incremental_encoder module_biss_encoder module_encoder_rem_14 module_serial_encoder module_shared_memory module_spi_master module_watchdog 

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the REM 16MT Service header **rem_16mt_service.h** in your app.

3. Instantiate the ports for the REM 16MT.

     REM 16MT needs a ``SPIPorts`` structure containing two clock blocks and 4 1-bit ports for SPI.

4. Fill up the REM 16MT configuration structure.

     The functions use the same configuration structure as the :ref:`Position Feedback Module <module_position_feedback>`.
     You need to fill up all the generic sensor parameters especially ``tile_usec`.
     And fill up the REM 16MT specific parameters.

5. At your IF2 tile, You can use the functions to read REM 16MT data.
    .. code-block:: c
    
        #include <CoreC2X.bsp>   			//Board Support file for SOMANET Core C2X device 
        #include <Drive1000-rev-c4.bsp>     //Board Support file for SOMANET Drive module 
                                            //(select your board support files according to your device)
        
        // 2. Include the REM 16MT Service header **rem_16mt_service.h** in your app.
        #include <rem_16_service.h>
        
        // 3.Instantiate the ports for the REM 16MT.
        SPIPorts spi_ports = SOMANET_DRIVE_SPI_PORTS;

        int main(void)
        {
            par
            {
                on tile[IF2_TILE]:
                {
                    // 4. Fill up the REM 16MT configuration structure.
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.polarity    = NORMAL_POLARITY;
                    position_feedback_config.pole_pairs  = POLE_PAIRS;
                    position_feedback_config.tile_usec   = IF2_TILE_USEC;
                    position_feedback_config.offset      = 0;

                    position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;
                    
                    // 5. Use the functions to read REM 16MT data.
                    // initialize the sensor
                    rem_16mt_init(spi_ports, position_feedback_config);
                    
                    // read REM 16MT data
                    int status, count, singleturn_filtered, singleturn_raw, timestamp;
                    { status, count, singleturn_filtered, singleturn_raw, timestamp } = rem_16mt_init(spi_ports, position_feedback_config.tile_usec);
                    
                    //reset REM 16MT position to zero
                    rem_16mt_write(spi_ports, REM_16MT_CONF_NULL, 0, 0, position_feedback_config.tile_usec)
                    
                    //write REM 16MT filter setting
                    rem_16mt_write(spi_ports, REM_16MT_CONF_FILTER, 0x02, 8, position_feedback_config.tile_usec)
                    
                    
                }
            }

            return 0;
        }

API
===

Definitions
-----------

.. doxygendefine:: DEFAULT_SPI_CLOCK_DIV
.. doxygendefine:: SPI_MASTER_MODE
.. doxygendefine:: SPI_MASTER_SD_CARD_COMPAT
.. doxygendefine:: REM_16MT_TIMEOUT
.. doxygendefine:: REM_16MT_POLLING_TIME
.. doxygendefine:: REM_16MT_CTRL_RESET
.. doxygendefine:: REM_16MT_CONF_DIR
.. doxygendefine:: REM_16MT_CONF_NULL
.. doxygendefine:: REM_16MT_CONF_PRESET
.. doxygendefine:: REM_16MT_CONF_STPRESET
.. doxygendefine:: REM_16MT_CONF_MTPRESET
.. doxygendefine:: REM_16MT_CONF_FILTER
.. doxygendefine:: REM_16MT_CALIB_TBL_SIZE
.. doxygendefine:: REM_16MT_CALIB_TBL_POINT
.. doxygendefine:: REM_16MT_CTRL_SAVE

Types
-----

.. doxygenstruct:: REM_16MTConfig
.. doxygenstruct:: PositionFeedbackConfig
.. doxygenstruct:: SPIPorts

Functions
--------

.. doxygenfunction:: rem_16mt_init
.. doxygenfunction:: rem_16mt_read
.. doxygenfunction:: rem_16mt_read
.. doxygenfunction:: rem_16mt_write


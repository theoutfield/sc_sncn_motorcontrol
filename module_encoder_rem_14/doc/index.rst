.. _module_encoder_rem_14:

=====================
REM 14 Encoder Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides functions to read  data coming from a REM 14 Encoder.

Those functions are used in :ref:`Serial Encoder Module <module_serial_encoder>` itself used by :ref:`Position Feedback Module <module_position_feedback>` to create a service for reading a REM 14 encoder.

The functions should always run over an **IF2 Tile** so it can access the ports to
your SOMANET Drive module.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_rem_14>`_


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`REM 14 Sensor Demo <app_test_rem_14`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = configuration_parameters module_adc module_encoder_rem_14 lib_bldc_torque_control module_board-support module_hall_sensor module_utils module_position_feedback module_pwm module_incremental_encoder module_biss_encoder module_encoder_rem_16mt module_serial_encoder module_shared_memory module_spi_master module_watchdog 

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the REM 14 Service header **rem_14_service.h** in your app.

3. Instantiate the ports for the REM 14.

     REM 14 needs a ``SPIPorts`` structure containing two clock blocks and 4 1-bit ports for SPI.

4. Fill up the REM 14 configuration structure.

     The functions use the same configuration structure as the :ref:`Position Feedback Module <module_position_feedback>`.
     You need to fill up all the generic sensor parameters especially ``tile_usec`.
     And fill up the REM 14 specific parameters.

5. At your IF2 tile, You can use the functions to read REM 14 data.
    .. code-block:: c

        #include <CoreC2X.bsp>   			//Board Support file for SOMANET Core C2X device 
        #include <Drive1000-rev-c4.bsp>     //Board Support file for SOMANET Drive module 
                                            //(select your board support files according to your device)

        // 2. Include the REM 14 Service header **rem_14_service.h** in your app.
        #include <rem_14_service.h>
        
        // 3.Instantiate the ports for the REM 14.
        SPIPorts spi_ports = SOMANET_DRIVE_SPI_PORTS;

        int main(void)
        {
            par
            {
                on tile[IF2_TILE]:
                {
                    // 4. Fill up the REM 14 configuration structure.
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.polarity    = NORMAL_POLARITY;
                    position_feedback_config.pole_pairs  = POLE_PAIRS;
                    position_feedback_config.tile_usec   = IF2_TILE_USEC;
                    position_feedback_config.offset      = 0;

                    position_feedback_config.rem_14_config.hysteresis     = REM_14_SENSOR_HYSTERESIS ;
                    position_feedback_config.rem_14_config.noise_setting  = REM_14_SENSOR_NOISE;
                    position_feedback_config.rem_14_config.dyn_angle_comp = REM_14_SENSOR_DAE;
                    position_feedback_config.rem_14_config.abi_resolution = REM_14_SENSOR_ABI_RES;
                    
                    // 5. Use the functions to read REM 14 data.
                    // initialize the sensor
                    initRotarySensor(spi_ports, position_feedback_config);
                    
                    // read REM 14 data
                    position = readRotarySensorAngleWithCompensation(spi_ports, position_feedback_config.tile_usec);
                }
            }

            return 0;
        }

API
===

Definitions
-----------

.. doxygendefine:: DEFAULT_SPI_CLOCK_DIV
.. doxygendefine:: REM_14_POLLING_TIME
.. doxygendefine:: REM_14_EXECUTING_TIME
.. doxygendefine:: REM_14_SAVING_TIME
.. doxygendefine:: REM_14_WIDTH_INDEX_PULSE
.. doxygendefine:: REM_14_FACTORY_SETTINGS
.. doxygendefine:: REM_14_UVW_ABI
.. doxygendefine:: REM_14_DATA_SELECT
.. doxygendefine:: REM_14_PWM_CONFIG

Types
-----

.. doxygenenum:: REM_14_ABIResolution
.. doxygenenum:: REM_14_Noise
.. doxygenenum:: REM_14_DynAngleComp
.. doxygenenum:: REM_14_Hysteresis
.. doxygenstruct:: REM_14Config
.. doxygenstruct:: PositionFeedbackConfig
.. doxygenstruct:: SPIPorts

Functions
--------

.. doxygenfunction:: initRotarySensorInterface
.. doxygenfunction:: initRotarySensor
.. doxygenfunction:: readZeroPosition
.. doxygenfunction:: readNumberPolePairs
.. doxygenfunction:: readRedundancyReg
.. doxygenfunction:: readProgrammingReg
.. doxygenfunction:: readCORDICMagnitude
.. doxygenfunction:: readRotaryDiagnosticAndAutoGainControl
.. doxygenfunction:: readRotarySensorError
.. doxygenfunction:: readRotarySensorAngleWithoutCompensation
.. doxygenfunction:: readRotarySensorAngleWithCompensation
.. doxygenfunction:: writeSettings
.. doxygenfunction:: writeZeroPosition
.. doxygenfunction:: writeNumberPolePairs


.. _app_test_position_feedback:

=================================
Position Feedback Service Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app is showing the use of the :ref:`Position Feedback Module <module_position_feedback>`.
For that, it implements a simple app that starts the Position Feedback Service to sensors and shows over **XScope** the velocity and position.
There are also commands to change the sensor type and restart the service.

* **Min. Nr. of cores**: 4
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_test_position_feedback/>`_

Quick How-to
============

1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your sensor, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`.
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the app. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropriate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. Again in your **main.xc**, set the configuration for the Position feedback Service and you sensor.

    .. code-block:: c

            on tile[IFM_TILE]:
            /* Position feedback service */
            {
                //set default parameters
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.pole_pairs  = POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = 0;

                position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config.biss_config.busy = BISS_BUSY;
                position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                position_feedback_config.rem_14_config.hysteresis     = REM_14_SENSOR_HYSTERESIS ;
                position_feedback_config.rem_14_config.noise_setting  = REM_14_SENSOR_NOISE;
                position_feedback_config.rem_14_config.dyn_angle_comp = REM_14_SENSOR_DAE;
                position_feedback_config.rem_14_config.abi_resolution = REM_14_SENSOR_ABI_RES;

                position_feedback_config.qei_config.index_type  = QEI_SENSOR_INDEX_TYPE;
                position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;
                position_feedback_config.qei_config.port_number = QEI_SENSOR_PORT_NUMBER;

                position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                position_feedback_config.gpio_config[0] = GPIO_INPUT_PULLDOWN;
                position_feedback_config.gpio_config[1] = GPIO_OUTPUT;
                position_feedback_config.gpio_config[2] = GPIO_OUTPUT;
                position_feedback_config.gpio_config[3] = GPIO_OUTPUT;

                PositionFeedbackConfig position_feedback_config_2;
                position_feedback_config_2 = position_feedback_config;

                //set sensor 1 parameters
                position_feedback_config.sensor_type = REM_16MT_SENSOR;
                position_feedback_config.resolution  = REM_16MT_SENSOR_RESOLUTION;
                position_feedback_config.velocity_compute_period = REM_16MT_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                //set sensor 1 parameters
                position_feedback_config_2.sensor_type = BISS_SENSOR;
                position_feedback_config_2.resolution  = BISS_SENSOR_RESOLUTION;
                position_feedback_config.velocity_compute_period = BISS_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config_2.sensor_function = SENSOR_FUNCTION_FEEDBACK_ONLY;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }

7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

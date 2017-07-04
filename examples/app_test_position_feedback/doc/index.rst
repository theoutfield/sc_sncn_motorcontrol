.. _app_test_position_feedback:

=================================
Position Feedback Service Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app is showing the use of the :ref:`Position Feedback Module <module_position_feedback>`.
For that, it implements a simple app that starts the Position Feedback Service with two sensors and shows over **XScope** the velocity and position and other info.
There are also commands to change the sensor type and restart the service.

The data displayed over XScope is:
      - Position 1: the position of sensor 1
      - Angle 1: the electrical angle of sensor 1
      - Velocity 1: the velocity of sensor 1
      - Position 2: the position of sensor 2
      - Angle 2: the electrical angle of sensor 2
      - Velocity 2: the velocity of sensor 2
      - Position Shared Memory: the position from the shared memory, the one used for motion control
      - Position additional Shared Memory: the additional position from the shared memory, the one use for display only
      - Angle Shared Memory: the electrical angle from the shared memory, used for commutation
      - Velocity Shared Memory: the velocity from the shared memory
      - GPIO 0: the value of the GPIO port 0

This app has a console interface to send commands to demonstrate how to change sensors at run time.

The commands are:
 - a [number]: set first sensor type
 - b [number]: set second sensor type
 - ar [number]: set first sensor resolution (ticks per turn)
 - av [number]: set first sensor velocity compute period in microseconds
 - ap [number]: set first sensor port number (1 or 2)
 - (same commands with 'b' for sensor 2)
 - e: exit and restart the position feedback service

The new sensor types are taken into account when the service is restarted

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

     The default configuration sets all parameters using ``defines`` from ``sensor_config.h``. You can edit this file or change the values directly in the ``main.xc``.

     You can change the sensor types for the sensor 1 and 2 by changing ``position_feedback_config_1.sensor_type`` and ``position_feedback_config_2.sensor_type``. Don't forget to also set the correponding ``resolution`` and ``velocity_compute_period``. Those can also be changed at run time.

    .. code-block:: c

            on tile[IFM_TILE]:
            /* Position feedback service */
            {
                //set default parameters
                PositionFeedbackConfig position_feedback_config_1;
                position_feedback_config_1.polarity    = NORMAL_POLARITY;
                position_feedback_config_1.pole_pairs  = POLE_PAIRS;
                position_feedback_config_1.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config_1.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config_1.offset      = 0;

                position_feedback_config_1.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config_1.biss_config.filling_bits = BISS_FILLING_BITS;
                position_feedback_config_1.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config_1.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                position_feedback_config_1.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config_1.biss_config.busy = BISS_BUSY;
                position_feedback_config_1.biss_config.clock_port_config = BISS_CLOCK_PORT;
                position_feedback_config_1.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                position_feedback_config_1.rem_16mt_config.filter = REM_16MT_FILTER;

                position_feedback_config_1.rem_14_config.hysteresis     = REM_14_SENSOR_HYSTERESIS ;
                position_feedback_config_1.rem_14_config.noise_setting  = REM_14_SENSOR_NOISE;
                position_feedback_config_1.rem_14_config.dyn_angle_comp = REM_14_SENSOR_DAE;
                position_feedback_config_1.rem_14_config.abi_resolution = REM_14_SENSOR_ABI_RES;

                position_feedback_config_1.qei_config.index_type  = QEI_SENSOR_INDEX_TYPE;
                position_feedback_config_1.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;
                position_feedback_config_1.qei_config.port_number = QEI_SENSOR_PORT_NUMBER;
                position_feedback_config.qei_config.ticks_lost_threshold = QEI_SENSOR_TICKS_LOST;

                position_feedback_config_1.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                position_feedback_config_1.gpio_config[0] = GPIO_INPUT_PULLDOWN;
                position_feedback_config_1.gpio_config[1] = GPIO_OUTPUT;
                position_feedback_config_1.gpio_config[2] = GPIO_OUTPUT;
                position_feedback_config_1.gpio_config[3] = GPIO_OUTPUT;

                PositionFeedbackConfig position_feedback_config_2;
                position_feedback_config_2 = position_feedback_config_1;

                //set sensor 1 parameters
                position_feedback_config_1.sensor_type = HALL_SENSOR;
                position_feedback_config_1.resolution  = HALL_SENSOR_RESOLUTION;
                position_feedback_config_1.velocity_compute_period = HALL_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config_1.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                //set sensor 1 parameters
                position_feedback_config_2.sensor_type = QEI_SENSOR;
                position_feedback_config_2.resolution  = QEI_SENSOR_RESOLUTION;
                position_feedback_config_2.velocity_compute_period = QEI_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config_2.sensor_function = SENSOR_FUNCTION_FEEDBACK_ONLY;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config_1, i_shared_memory[0], i_position_feedback_1,
                        position_feedback_config_2, i_shared_memory[1], i_position_feedback_2);
            }


7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

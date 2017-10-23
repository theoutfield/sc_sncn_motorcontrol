.. _app_test_biss:

=================================
BiSS Interface Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app is showing the use of the :ref:`BiSS Interface Module <module_biss>` with :ref:`Position Feedback Module <module_position_feedback>`.
For that, it implements a simple app that reads the output of an BiSS sensor and shows over **XScope** the read velocity and position.
It also displays the status of the BiSS sensor for debugging.

* **Min. Nr. of cores**: 2
* **Min. Nr. of tiles**: 1

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_test_biss/>`_

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

            on tile[IF2_TILE]:
            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = BISS_SENSOR;
                position_feedback_config.resolution  = BISS_SENSOR_RESOLUTION;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.velocity_compute_period = BISS_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.pole_pairs  = POLE_PAIRS;
                position_feedback_config.ifm_usec    = IF2_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = 0;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config.biss_config.busy = BISS_BUSY;
                position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, null, null, null, null,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
            
7. In parallel, the position/velocity and others status info are displayed with XScope.

    .. code-block:: c
        
        on tile[APP_TILE]:
        {
            timer t;
            unsigned int start_time, end_time;
            int count = 0;
            unsigned int angle = 0;
            int velocity = 0;
            unsigned int position = 0;
            unsigned int status = 0;

            while(1) {

                /* get position from BiSS Encoder */
                { count, position, status } = i_position_feedback.get_position();

                t :> start_time;
                /* get angle and velocity from BiSS Encoder */
                angle = i_position_feedback.get_angle();
                velocity = i_position_feedback.get_velocity();
                t :> end_time;


                if (!isnull(i_shared_memory)) {
                    UpstreamControlData upstream_control_data = i_shared_memory.read();
                    angle = upstream_control_data.angle;
                    count = upstream_control_data.position;
                    velocity = upstream_control_data.velocity;
                }

                xscope_int(COUNT, count);                           //absolute count
                xscope_int(POSITION, position);                     //singleturn position
                xscope_int(ANGLE, angle);                           //electrical angle
                xscope_int(VELOCITY, velocity);                     //velocity in rpm
                xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds
                xscope_int(STATUS_X100, status*100);                //error status
                
                switch(status) {
                case SENSOR_CHECKSUM_ERROR:
                    xscope_int(CRC_ERROR, 1000);    //CRC error
                    xscope_int(ERROR_BIT, 0);       //error bit
                    xscope_int(WARNING_BIT, 0);     //warning bit
                    break;
                case SENSOR_BISS_ERROR_BIT_ERROR:
                    xscope_int(CRC_ERROR, 0);       //CRC error
                    xscope_int(ERROR_BIT, 1000);    //error bit
                    xscope_int(WARNING_BIT, 0);     //warning bit
                    break;
                case SENSOR_BISS_WARNING_BIT_ERROR:
                    xscope_int(CRC_ERROR, 0);       //CRC error
                    xscope_int(ERROR_BIT, 0);       //error bit
                    xscope_int(WARNING_BIT, 1000);  //warning bit
                    break;
                case SENSOR_BISS_ERROR_AND_WARNING_BIT_ERROR:
                    xscope_int(CRC_ERROR, 0);       //CRC error
                    xscope_int(ERROR_BIT, 1000);    //error bit
                    xscope_int(WARNING_BIT, 1000);  //warning bit
                    break;
                default:
                    xscope_int(CRC_ERROR, 0);       //CRC error
                    xscope_int(ERROR_BIT, 0);       //error bit
                    xscope_int(WARNING_BIT, 0);     //warning bit
                    break;
                }

                delay_milliseconds(1);
            }
        }


8. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

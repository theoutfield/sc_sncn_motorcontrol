.. _module_position_feedback:

=====================
Position Feedback Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service which starts one or two Position Sensor Services among BiSS, Contelec, AMS, Hall and QEI.

The service takes as parameters:
 - ports for the different sensors
 - one config structure for each of the sensor services started (so 2 structures).
 - client interfaces to the shared memory for each of the sensor services (2).
 - server interfaces for the sensor services (2).

The service will initialize the ports and detect wrong configuration (for exemple starting two sensor services using the same ports). Then it will start one or two services. The second service is optional and will not be started if no config structure or server interface is provided it. The type sensor service started are set using the ``sensor_type`` parameter of the config structure.

The services have an ``exit()`` interface call which will end their loop and the Position Feedback Service will restart and start a new service. Again according to the ``sensor_type`` parameter, so if it did not change the same services will restart.

The BiSS Service should always run over an **IFM Tile** so it can access the ports to
your SOMANET IFM device.


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`Position feedback Demo <app_test_position_feedback>`, which illustrates the use of this module.

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile. The Position Feedback Service needs all the sensor modules it supports (BiSS, Contelec, AMS, Hall and QEI).

    ::

        USED_MODULES = module_ams_rotary_sensor module_biss module_board-support module_contelec_encoder module_hall module_memory_manager module_misc module_position_feedback module_qei module_spi_master

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules.
          This will help solving internal dependency issues.

2. Include the Position Feedback Service header **position_feedback_service.h** in your app.

3. Instantiate the ports needed for the sensors.

4. Inside your main function, instantiate the interfaces array for the Service-Clients communication.

5. At your IFM tile, instantiate the Service. For that, first you will have to fill up your Service configuration.

6. At whichever other core, now you can perform calls to the Position Feddback Service through the interfaces connected to it. Or if it is enabled you can read the position using the shared memory.

    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        #include <position_feedback_service.h>
       
        HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
        SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
        QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

        int main(void)
        {
            interface PositionFeedbackInterface i_position_feedback_1[3];
            interface PositionFeedbackInterface i_position_feedback_2[3];
            interface shared_memory_interface i_shared_memory[3];

            par
            {

                on tile[IFM_TILE]: par {
                    memory_manager(i_shared_memory, 3);

                    /* Position feedback service */
                    {
                        PositionFeedbackConfig position_feedback_config;
                        position_feedback_config.sensor_type = HALL_SENSOR;

                        position_feedback_config.biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                        position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                        position_feedback_config.biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                        position_feedback_config.biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                        position_feedback_config.biss_config.status_length = BISS_STATUS_LENGTH;
                        position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                        position_feedback_config.biss_config.pole_pairs = 2;
                        position_feedback_config.biss_config.polarity = BISS_POLARITY;
                        position_feedback_config.biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                        position_feedback_config.biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                        position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                        position_feedback_config.biss_config.max_ticks = BISS_MAX_TICKS;
                        position_feedback_config.biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                        position_feedback_config.biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;
                        position_feedback_config.biss_config.enable_push_service = PushAll;

                        position_feedback_config.contelec_config.filter = CONTELEC_FILTER;
                        position_feedback_config.contelec_config.polarity = CONTELEC_POLARITY;
                        position_feedback_config.contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                        position_feedback_config.contelec_config.offset = CONTELEC_OFFSET;
                        position_feedback_config.contelec_config.pole_pairs = 2;
                        position_feedback_config.contelec_config.timeout = CONTELEC_TIMEOUT;
                        position_feedback_config.contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                        position_feedback_config.contelec_config.enable_push_service = PushAll;

                        position_feedback_config.hall_config.pole_pairs = 2;
                        position_feedback_config.hall_config.enable_push_service = PushAll;

                        position_feedback_config.qei_config.ticks_resolution = 1000;
                        position_feedback_config.qei_config.index_type = QEI_WITH_INDEX;
                        position_feedback_config.qei_config.sensor_polarity = 1;
                        position_feedback_config.qei_config.signal_type = QEI_RS422_SIGNAL;
                        position_feedback_config.qei_config.enable_push_service = PushPosition;

                        PositionFeedbackConfig position_feedback_config_2;
                        position_feedback_config_2 = position_feedback_config;
                        position_feedback_config_2.sensor_type = BISS_SENSOR;

                        position_feedback_service(hall_ports, qei_ports, spi_ports,
                                                  position_feedback_config, i_shared_memory[0], i_position_feedback_1,
                                                  position_feedback_config_2, null, i_position_feedback_2);
                    }
                }
            }

            return 0;
        }



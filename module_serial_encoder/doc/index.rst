.. _module_serial_encoder:

=====================
Serial Encoder Module
=====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from an Encoder communicating over a serial protocol like BiSS or SPI.

This service can run independently but is meant to be used by the :ref:`Position Feedback Module <module_position_feedback>` that is why it uses the same communication interface.

The Service should always run over an **IF2 tile** so it can access the ports to
your SOMANET IFM device.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_serial_encoder>`_

.. image:: images/core-diagram-serial-encoder.png
   :width: 50%


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
.. seealso:: You might find useful the :ref:`Position feedback Demo <app_test_position_feedback>` example app, which illustrates the use of this module. 

1. First, add all the :ref:`SOMANET Motion Control <somanet_motion_control>` modules to your app Makefile.

    ::

        USED_MODULES = configuration_parameters module_biss_encoder lib_bldc_torque_control module_board-support module_hall_sensor module_shared_memory module_utils module_position_feedback module_serial_encoder module_encoder_rem_14 module_encoder_rem_16mt module_incremental_encoder module_spi_master

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependency issues.

2. Include the Encoder Service header **serial_encoder_service.h.h** in your app. 

3. Instantiate the ports where the Service will be reading the Encoder Sensor feedback signals. 

     Depending on the configuration the Serial Encoder service uses ``qei_hall_port`` ``1`` or ``2``, ``p_hall_enc_select``, a GPIO port and a clock block (in ``SPI ports``) for BiSS.
     ``SPI ports`` and GPIO ports for SPI.
     The ports structures are defined in ``position_feedback_service.h``.

4. Inside your main function, instantiate the interfaces array for the Service-Clients communication.

5. Optionally, instantiate the shared memory interface.

6. At your IF2 tile, instantiate the Service. For that, first you will have to fill up your Service configuration.

     The service uses the same configuration structure as the :ref:`Position Feedback Module <module_position_feedback>`.
     You need to fill up all the generic sensor parameters especially ``ifm_usec``, ``resolution``, ``velocity_compute_period`` and ``sensor_function``.
     And you need to fill up sensor specific parameters depending on the sensor you want to use (BiSS, REM 14, REM 16MT).

7. At whichever other core, now you can perform calls to the Encoder Service through the interfaces connected to it.

    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)
                                        
        // 2. Include the Hall Service header
        #include <serial_encoder_service.h>
       
        // 3. Instantiate the ports needed for the sensor.
        QEIHallPort qei_hall_port_1 = SOMANET_DRIVE_HALL_PORTS;
        QEIHallPort qei_hall_port_2 = SOMANET_DRIVE_QEI_PORTS;
        HallEncSelectPort hall_enc_select_port = SOMANET_DRIVE_QEI_PORT_INPUT_MODE_SELECTION;
        SPIPorts spi_ports = SOMANET_DRIVE_SPI_PORTS;
        port ?gpio_port_0 = SOMANET_DRIVE_GPIO_D0;
        port ?gpio_port_1 = SOMANET_DRIVE_GPIO_D1;
        port ?gpio_port_2 = SOMANET_DRIVE_GPIO_D2;
        port ?gpio_port_3 = SOMANET_DRIVE_GPIO_D3;


        int main(void)
        {
            // 4. Instantiate the interfaces array for the Service-Clients communication.
            interface PositionFeedbackInterface i_position_feedback_1[3];
            
            // 5. Instantiate the shared memory interface.
            interface shared_memory_interface i_shared_memory[3];

            par
            {

                on tile[IF2_TILE]: par {
                    // 5. Start the shared memory service
                    shared_memory_service(i_shared_memory, 3);

                    // 6. Fill up your Service configuration and instantiate the Service. 
                    /* Position feedback service */
                    {
                        //set default parameters
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
                        
                        position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                        position_feedback_config.rem_14_config.hysteresis     = REM_14_SENSOR_HYSTERESIS ;
                        position_feedback_config.rem_14_config.noise_setting  = REM_14_SENSOR_NOISE;
                        position_feedback_config.rem_14_config.dyn_angle_comp = REM_14_SENSOR_DAE;
                        position_feedback_config.rem_14_config.abi_resolution = REM_14_SENSOR_ABI_RES;


                        position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                                                  position_feedback_config, i_shared_memory[0], i_position_feedback,
                                                  null, null, null);                                          
                    }
                }
                
                on tile[APP_TILE]:
                {
                    int count_1, position_1, angle_1, velocity_1;
                    int count_2, position_2, status_2, angle_2, velocity_2;
                    
                    // 7. Call to the Position Feddback Service through the interfaces connected to it.                
                    /* get position from Sensor 1 */
                    { count_1, position_1, void } = i_position_feedback_1[0].get_position();
                    angle_1 = i_position_feedback_1[0].get_angle();
                    velocity_1 = i_position_feedback_1[0].get_velocity();
                    
                    // 7. You can also read the position using the shared memory.
                    UpstreamControlData upstream_control_data = i_shared_memory[2].read();
                    angle_1 = upstream_control_data.angle;
                    count_1 = upstream_control_data.position;
                    velocity_1 = upstream_control_data.velocity;
                }
            }

            return 0;
        }

API
===

Types
-----
.. doxygenstruct:: PositionFeedbackConfig
.. doxygenstruct:: BISSConfig
.. doxygenstruct:: REM_14Config
.. doxygenstruct:: REM_16MTConfig
.. doxygenstruct:: QEIHallPort
.. doxygenstruct:: HallEncSelectPort
.. doxygenstruct:: SPIPorts

Service
--------

.. doxygenfunction:: serial_encoder_service


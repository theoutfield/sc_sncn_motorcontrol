.. _module_hall_sensor:

===========================
Hall Sensor Module
===========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from your 
Feedback Hall Sensor.

This service can run independently but is meant to be used by the :ref:`Position Feedback Module <module_position_feedback>` that is why it uses the same communication interface.

The Hall Service should always run over an **IF2 Tile** so it can access the ports to
your SOMANET Drive module.

.. image:: images/core-diagram-hall-interface.png
   :width: 50%


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
.. seealso:: You might find useful the :ref:`SOMANET Hall Effect Feedback Sensor Demo <app_test_hall>`, which illustrates the use of this module. 

1. First, add all the :ref:`SOMANET Motor Control <somanet_motion_control>` modules to your app Makefile.

    ::

        USED_MODULES = configuration_parameters module_biss_encoder lib_bldc_torque_control module_board-support module_hall_sensor module_shared_memory module_utils module_position_feedback module_incremental_encoder module_encoder_rem_14 module_encoder_rem_16mt module_serial_encoder module_spi_master

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependency issues.

2. Include the Hall Service header **hall_service.h** in your app. 

3. Instantiate the ports where the Service will be reading the Hall Sensor feedback signals.

     The Hall service uses ``qei_hall_port`` ``1`` or ``2`` depending on the configuration.
     The ports structures are defined in ``position_feedback_service.h``.

4. Inside your main function, instantiate the interfaces array for the Service-Clients communication.

5. Optionally, instantiate the shared memory interface.

6. At your IF2 tile, instantiate the Service. For that, first you will have to fill up your Service configuration.

     The Hall sensor has only one specific parameter ``hall_config.port_number`` the port number used.
     You still need to fill up all the generic sensor parameters especially ``tile_usec``, ``resolution``, ``velocity_compute_period`` and ``sensor_function``.

7. At whichever other core, now you can perform calls to the Position Feedback Service through the interfaces connected to it. Or if it is enabled you can read the position using the shared memory.

    .. code-block:: c

        #include <CoreC2X.bsp>   			//Board Support file for SOMANET Core C2X device 
        #include <Drive1000-rev-c4.bsp>     //Board Support file for SOMANET Drive module 
                                            //(select your board support files according to your device)
                                        
        // 2. Include the Hall Service header
        #include <hall_service.h>
       
        // 3. Instantiate the ports needed for the sensor.
        QEIHallPort qei_hall_port_1 = SOMANET_DRIVE_HALL_PORTS;
        QEIHallPort qei_hall_port_2 = SOMANET_DRIVE_QEI_PORTS;

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
                        position_feedback_config.polarity    = NORMAL_POLARITY;
                        position_feedback_config.pole_pairs  = POLE_PAIRS;
                        position_feedback_config.tile_usec   = IF2_TILE_USEC;
                        position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                        position_feedback_config.offset      = 0;
                        position_feedback_config.sensor_type = HALL_SENSOR;
                        position_feedback_config.resolution  = HALL_SENSOR_RESOLUTION;
                        position_feedback_config.velocity_compute_period = HALL_SENSOR_VELOCITY_COMPUTE_PERIOD;
                        position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                        position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                        position_feedback_service(qei_hall_port_1, qei_hall_port_2, null, null, null, null, null, null,
                                position_feedback_config, i_shared_memory[0], i_position_feedback_1,
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

.. doxygenstruct:: HallConfig
.. doxygenstruct:: PositionFeedbackConfig
.. doxygenstruct:: QEIHallPort

Service
-------

.. doxygenfunction:: hall_service


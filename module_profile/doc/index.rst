.. _module_profile:

=======================
SOMANET Profile Module
=======================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module contains utilities to generate Position, Velocity and Torque Profile ramps on top of
a Motor Control PID Controller.

Profile ramps ensure a smooth transition between set points on a control loop. In some cases,
where 2 set points are not close enough, it is mandatory the use of profiles for apropiate motion of
the motor.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_profile>`_

How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
.. seealso:: 
    You might find useful the **BLDC/Brushed DC Position, Velocity and Torque Control Demo** example apps, which illustrate the use of this module: 
    
    * :ref:`BLDC Position Control Demo <bldc_position_control_demo>`
    * :ref:`BLDC Position Control Demo <bldc_velocity_control_demo>`
    * :ref:`BLDC Torque Control Demo <bldc_torque_control_demo>`
    * :ref:`Brushed DC Position Control Demo <brushed_dc_position_control_demo>`
    * :ref:`Brushed DC Velocity Control Demo <brushed_dc_velocity_control_demo>`

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_profile module_hall module_pwm_symmetrical module_adc module_ctrl_loops module_misc module_motorcontrol module_gpio module_qei module_watchdog module_board-support

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Properly instanciate a **Position Control Loop Service**.

3. Include the Profiler header **profile_control.h** in your app. 

4. Over whichever other core you can perform calls to the profiler. Do not forget to first fill up the configuration and initialize your profiler.


.. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <pwm_service.h>
        #include <hall_service.h>
        #include <watchdog_service.h>
        #include <motorcontrol_service.h>
        #include <position_ctrl_service.h> 
        #include <profile_control.h> // 3

        PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
        WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
        HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
        FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

        int main(void)
        {

            chan c_pwm_ctrl;            
            interface WatchdogInterface i_watchdog[2];
            interface MotorcontrolInterface i_motorcontrol[5];
            interface HallInterface i_hall[5];
            interface PositionControlInterface i_position_control[3]; 

            par
            {
                on tile[APP_TILE]:
                {
                    ProfilerConfig profiler_config;
                    profiler_config.polarity = 1;
                    profiler_config.max_position = 128000000;
                    profiler_config.min_position = -128000000;
                    profiler_config.max_velocity = 5000;
                    profiler_config.max_acceleration = 10000;
                    profiler_config.max_deceleration = 10000;

                    init_position_profiler(profiler_config, i_position_control[0]);
                    set_profile_position(50000, 500, 5000, 5000, i_position_control); // 4
                }
                on tile[APP_TILE]:
                {
                    ControlConfig position_control_config;
                    position_control_config.feedback_sensor = HALL_SENSOR;
                    position_control_config.Kp_n = 100;    
                    position_control_config.Ki_n = 10;    
                    position_control_config.Kd_n = 0;    
                    position_control_config.control_loop_period = 60;

                    position_control_service(position_control_config, i_hall[1], null, i_motorcontrol[0],
                                                i_position_control); 
                }

                on tile[IFM_TILE]:
                {
                    par
                    {
                        pwm_service(pwm_ports, c_pwm_ctrl);

                        watchdog_service(wd_ports, i_watchdog);

                        {
                            HallConfig hall_config;
                            hall_config.pole_pairs = 1;

                            hall_service(hall_ports, hall_config, i_hall);
                        }

                        {
                            MotorcontrolConfig motorcontrol_config;
                            motorcontrol_config.motor_type = BLDC_MOTOR;
                            motorcontrol_config.commutation_sensor = HALL_SENSOR;
                            motorcontrol_config.bldc_winding_type = STAR_WINDING;
                            motorcontrol_config.hall_offset[0] = 0;
                            motorcontrol_config.hall_offset[1] = 0;
                            motorcontrol_config.commutation_loop_period = 60;

                            motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                                    c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
                        }
                    }
                }
            }

            return 0;
        }

.. note:: Similary to this example, you can repeat the same steps for Velocity and Torque Profilers. Each Profiler would require different parameters of the **ProfilerConfig**.

API
===

Global Types
------------

.. doxygenstruct:: ProfilerConfig

Position Profiler
-----------------

.. doxygenfunction:: init_position_profiler
.. doxygenfunction:: set_profile_position

Velocity Profiler
-----------------

.. doxygenfunction:: init_velocity_profiler
.. doxygenfunction:: set_profile_velocity

Torque Profiler
---------------

.. doxygenfunction:: init_torque_profiler
.. doxygenfunction:: set_profile_torque

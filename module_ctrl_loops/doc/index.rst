.. _module_ctrl_loops:

=============================
SOMANET Control Loops Module 
=============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a three Services to perform Position, Velocity and Torque
PID control over a Motor Control Service. Up to 3 clients could control the Services
through interfaces.

The Position, Velocity and Torque Control Services are intended to run at a **100MHz Reference Frequency**,
therefore they must be instanciated in a different tile from the remaining Motor Control Services.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_ctrl_loops>`_

How to use (eg. Position Control Service)
=========================================


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

        USED_MODULES = module_ctrl_loops module_motorcontrol module_pwm_symmetrical module_adc module_hall module_misc module_profile module_qei module_gpio module_watchdog module_board-support

    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
              This will help solving internal dependency issues.

2. Properly instanciate a :ref:`Motor Control Service <module_motorcontrol>`.

3. Include the Position Control Service header **position_ctrl_service.h** in your app. 

4. Inside your main function, instanciate the interfaces array for the Service-Clients communication.

5. Outside your IFM tile, instanciate the Service. For that, first you will have to fill up your Service configuration and provide interfaces to your control feedback sensor Service and Motor Control Service.

6. At whichever other core, now you can perform calls to the Control Service through the interfaces connected to it. Do not forget to initialize it first.

    .. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <pwm_service.h>
        #include <hall_service.h>
        #include <watchdog_service.h>
        #include <motorcontrol_service.h>
        #include <position_ctrl_service.h> // 3

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
            interface PositionControlInterface i_position_control[3]; // 4

            par
            {
                on tile[APP_TILE]:
                {
                    init_position_control(i_position_control[0]);
                    i_position_control[0].set_position(4095); // 6
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
                                                i_position_control); // 5
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

    .. note:: Similary to this example, you can repeat the same steps for Velocity and Torque Control Loops. 

API
===

Definitions
-------------

.. doxygendefine:: PID_DENOMINATOR

Global Types
-------------

.. doxygenstruct:: ControlConfig

Position Control Loop
---------------------

Position Control Service
````````````````````````

.. doxygenfunction:: init_position_control
.. doxygenfunction:: position_control_service
.. doxygenfunction:: position_limit

Position Control Interface
``````````````````````````

.. doxygeninterface:: PositionControlInterface


Velocity Control Loop
---------------------

Velocity Service
````````````````

.. doxygenfunction:: init_velocity_control
.. doxygenfunction:: velocity_control_service
.. doxygenfunction:: max_speed_limit

Velocity Interface
``````````````````

.. doxygeninterface:: VelocityControlInterface

Torque Control Loop
-------------------

Torque Service
````````````````
.. doxygenfunction:: init_torque_control
.. doxygenfunction:: torque_control_service
.. doxygenfunction:: torque_limit

Torque Interface
````````````````
.. doxygeninterface:: TorqueControlInterface

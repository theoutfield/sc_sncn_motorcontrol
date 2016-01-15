.. _offset_commutation_tuning_demo:

============================================
Helper for finding your Hall sensor offsets
============================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

It is common that Hall Effect sensors used for position feedback are not perfectly placed physically along the windings. This strongly affects the commutation efficiency of your motor. In order to avoid such loss of efficiency we can configure at the :ref:`Motor Control Service <module_motorcontrol>` the offsets to compensate the Hall sensors error on clock-wise and counter-clock-wise spin. The purpose of this app (app_demo_offset_commutation_tuning) is helping you to find such offsets.

The motor will commutate at a certain voltage (either positive, clockwise spin, or negative, counter-clockwise spin) and the current at the phases will be displayed over XScope. At the same time, the user could set different offsets over the console and see the effect of it on the amplitude of the phase currents. 

* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 3

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_offset_commutation_tuning/>`_

Quick How-to
============

1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Hall sensor, motor phases, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_offset_commutation_tuning**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. Set parameters for your :ref:`Motor Control Service <module_motorcontrol>`. Just the **bldc_winding_type**, hall_offset[0] (CW)  and hall_offset[1] (CCW) are relevant for this app. Remaining parameters could stay like this:

.. code-block:: C

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_sensor = HALL_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = 0;
                    motorcontrol_config.hall_offset[1] = 0;
                    motorcontrol_config.commutation_loop_period = 60;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
                }

7.  Define a low voltage value to start with, 1000 should be fine. Since it is a positive value, it will help you finding the clockwise offset.

.. code-block:: C
        #define VOLTAGE 1000

8. :ref:`Run the application enabling XScope <running_an_application>`.

9. Try different offset values until you get the minimum possible amplitude of currents.

10. Repeat steps **7**, **8** and **9** for negative voltage values, so you find the counter-clockwise error offset. Also you can try higher voltage values (up to 4000) to obtain finer results.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

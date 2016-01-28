.. _brushed_dc_drive_demo:

=====================================
SOMANET Brushed DC Motor Control Demo
=====================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_brushed_dc) is showing the use of the :ref:`Motor Control Module <module_motorcontrol>` over a Brushed DC motor. For that, it implements a simple app that drives your motor without any position feedback sensor input.

* **Minimum Number of Cores**: 4
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_brushed_dc/>`_

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your motor phases (A and B), power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_brushed_dc**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the configuration <motor_configuration_label>` for Motor Control.

.. code-block:: C

                /* Motor drive service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BDC_MOTOR;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config, c_pwm_ctrl, null, null, i_watchdog[0],
                                                i_motorcontrol);
                }

7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.


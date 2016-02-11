.. _offset_commutation_tuning_demo:

============================================
Helper for finding your sensor offsets
============================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

It is common that sensors used for position feedback are not perfectly placed physically along the windings. This strongly affects the commutation efficiency of your motor. In order to avoid such loss of efficiency we can configure at the :ref:`Motor Control Service <module_motorcontrol>` the sensor offset and the offsets to compensate the error on clock-wise and counter-clock-wise spin. The purpose of this app (app_demo_offset_commutation_tuning) is helping you to find such offsets.

The motor will commute at a certain voltage and the current at the phases will be displayed over XScope. At the same time, the user could set different offsets over the console and see the effect of it on the amplitude of the phase currents.

The app uses commands to set the offsets and voltage over the console:

- a
    Automatically find the sensor offset, the clockwise or counterclockwise offsets are set to 0 and 2048 (half a turn)
- s VALUE
    Set the sensor offset to VALUE
- VALUE
    Set the clockwise or counterclockwise (depending on the voltage sign and winding type) offset to VALUE
- d
    Reverse sensor direction, use this when the phases are wired incorrectly and the motor is not moving
- v VALUE
    Set the voltage to VALUE
- r
    Reverse the voltage
- p
    Print the offsets, sensor polarity and voltage


* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 3

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_offset_commutation_tuning/>`_

Quick How-to
============

#. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
#. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Hall sensor, motor phases, power supply cable, and XTAG. Power up!
#. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
#. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
#. Open the **main.xc** within  the **app_demo_offset_commutation_tuning**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. Set parameters for your :ref:`Motor Control Service <module_motorcontrol>`. Just the **bldc_winding_type**, hall_offset[0] (CW)  and hall_offset[1] (CCW) are relevant for this app. Remaining parameters could stay like this:

.. code-block:: C

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = 0;
                    motorcontrol_config.hall_offset[1] = 0;
                    motorcontrol_config.commutation_loop_period = COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, null, null, null, i_ams[0], i_watchdog[0], i_motorcontrol);
                }

7. Set parameters for your sensor. For AMS sensor it is in ``ams_service.h``. The most important parameters are the sensor offset and polarity.

.. code-block:: C

                #define AMS_OFFSET      5167
                #define AMS_POLARITY    AMS_POLARITY_NORMAL

8.  Define a low voltage value to start with, 1000 should be fine.

.. code-block:: C

        #define VOLTAGE 1000

9. :ref:`Run the application enabling XScope <running_an_application>`.

10. First try to set the offset automatically with the ``a`` command. If the motor is not turning try to change the sensor polarity with the ``d`` command and repeat the ``a`` command. This will find the sensor offset and set the clockwise or counterclockwise offsets to 0 and 2048 (half a turn). The motor should start turning.

   Adjust the sensor offset with the ``s VALUE`` command to minimise the current consumption for the current direction.

   Reverse the voltage with the ``r`` command, the motor should turn is the other direction. Adjust the offset for this direction with the ``VALUE`` command (clockwise or counterclockwise offset depending on the voltage sign and winding type).

   You can change the voltage with the ``v VALUE`` command (up to 4000) to test the offsets at different velocity and obtain finer results.

   Finally you can print the offsets with the ``p`` command and edit your config files accordingly.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

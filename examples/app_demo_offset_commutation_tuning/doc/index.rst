.. _offset_commutation_tuning_demo:

======================================================
Helper for finding your sensor and commutation offsets
======================================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

It is common that sensors used for position feedback are not perfectly placed physically along the windings. This strongly affects the commutation efficiency of your motor. In order to avoid such loss of efficiency we can configure at the *Position Sensor Service* the sensor offset and at the :ref:`Motor Control Service <module_motorcontrol>` the commutation offsets to compensate the error on clock-wise and counter-clock-wise spin. The purpose of this app (app_demo_offset_commutation_tuning) is helping you to find such offsets.

The motor will commutate at a certain voltage and the current at the phases will be displayed over XScope. At the same time, the user could set different offsets over the console and see the effect of it on the amplitude of the phase currents.


* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 3

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_offset_commutation_tuning/>`_


Console commands
================

The app uses commands to set the offsets and voltage over the console:

- a
    Automatically find the sensor offset, the clockwise or counterclockwise commutation offsets are set to 0 and 2048 (half a turn). This is not very precise but should suffice to turn the motor.
- c
    Automatically tune the commutation offset, this works by searching the offset with minimum peak current consumption.
- s VALUE
    Set the sensor offset to VALUE
- VALUE
    Set the commutation offset to VALUE (clockwise or counterclockwise offset depending on the voltage sign and winding type)
- d
    Reverse motor polarity, use this when the the motor is not moving. It happens when the phases wiring or the position sensor polarity is changed.
- f
    Flip the clockwise and counterclockwise commutation offsets, this will change the direction of the motor.
- v VALUE
    Set the voltage to VALUE, accept negative values
- r
    Reverse the voltage
- p
    Print the offsets, sensor polarity and voltage


Quick How-to
============

#. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
#. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your position sensor, motor phases, power supply cable, and XTAG. Power up!

   .. important:: For safety please use a current limited power supply and always monitor the current consumption during the tuning procedure.

#. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`.
#. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
#. Edit **user_config.h** in **config_motor** to set the motor parameters. The importants parameters are the number of poles pairs, the winding type, the commutation sensor and the commutation offsets. For the first start leave the offsets to their default values.

   .. code-block:: C

                #define POLE_PAIRS                11
                #define BLDC_WINDING_TYPE         STAR_WINDING
                #define MOTOR_COMMUTATION_SENSOR  AMS_SENSOR
                #define COMMUTATION_OFFSET_CLK    0
                #define COMMUTATION_OFFSET_CCLK   2048

#. Set parameters for your position sensor. The most important parameters are the sensor offset and polarity. For the first start leave the default offset value. The sensor polarity will define the physical direction of your motor. You can use the test app of the position sensor to test which physical direction corresponds to a positive velocity.

   - For AMS sensor edit **ams_service.h** in **module_ams_rotary_sensor**:

     .. code-block:: C

                     #define AMS_OFFSET      0
                     #define AMS_POLARITY    AMS_POLARITY_NORMAL

   - For BiSS sensor edit **biss_service.h** in **module_biss**:

     .. code-block:: C

                     #define BISS_OFFSET_ELECTRICAL  0
                     #define BISS_POLARITY           BISS_POLARITY_NORMAL

   - For Hall sensor no parameters are needed.

#. Open the **main.xc** within  the **app_demo_offset_commutation_tuning**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

   .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

#. Set parameters for your :ref:`Motor Control Service <module_motorcontrol>` to use the values previously defined in **user_config.h**. The motor polarity depends on the wiring of the phases and the position sensor polarity.

   .. code-block:: C

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.polarity_type = NORMAL_POLARITY;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period = COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], null, i_biss[0], i_ams[0], i_watchdog[0], i_motorcontrol);
                }

#. Define a low voltage value to start with. The value depends on you motor, usually less than 1000. The value can be changed at run time in the app. Remember to use a current limited power supply and always monitor the current consumption.

   .. code-block:: C

                   #define VOLTAGE 1000

#. :ref:`Run the application enabling XScope <running_an_application>`.

#. The app start with ``0`` commutation voltage so the motor will not move and the current consumption should be low. Remember to use a current limited power supply and always monitor the current consumption.

   First try to set the offset automatically with the ``a`` command. If the motor is not turning and the current consumption is high try to change the motor polarity with the ``d`` command and repeat the ``a`` command. This will find the sensor offset and set the clockwise or counterclockwise commutation offsets to 0 and 2048 (half a turn) and the motor should start turning.

   With a positive voltage the motor should turn in the direction of positive velocity. If it is not the case you can change the direction by flipping the clockwise and counterclockwise commutation offsets with the ``f`` command.

   Fine tune the sensor commutation offset for the current direction. You could use the ``c`` command for auto tuning or the ``VALUE`` command to manually minimize the phases current. The offset is a 12 bit positive value so it wraps around at 4096. It means that if you want an offset of ``-100`` you enter ``3996``.

   Reverse the voltage with the ``r`` command, the motor should turn in the other direction. Fine tune the commutation offset for this direction with the ``c`` (auto tuning) or ``VALUE`` (manual tuning) command.

   You can change the voltage with the ``v VALUE`` command (up to 4000) to test and tune the offsets at a different velocity and obtain finer results.

   You can print all the current offsets with the ``p`` command.

   .. important:: When you have found all the offsets save them in your configuration files for your app:

                  - the motor configuration file **user_config.h**
                  - the sensor configuration file **ams_service.h** or **biss_service.h**
                  - the **main.c** of your app (for the motor polarity)

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

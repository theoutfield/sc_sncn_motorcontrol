.. _watchdog_driver_demo:

============================
SOMANET Watchdog Driver Demo
============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_watchdog) is showing the use of the :ref:`Watchdog Module <module_watchdog>`. For that, it implements a simple app that disables the Watchdog ticking while spinning the motor. After that it renables the Watchdog chip operation again.

* **Min. Nr. of cores**: 5
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_watchdog/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, motor phases, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_demo_watchdog**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

           .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Set the configuration for Motor Control and Hall Service.   (LINK TO HOW TO CONFIGURE) 

.. code-block:: C

        /* Hall sensor Service */
        {
            HallConfig hall_config;
            hall_config.pole_pairs = POLE_PAIRS;

            hall_service(hall_ports, hall_config, i_hall);
        }

        /* Motor Commutation Service */
        {
            MotorcontrolConfig motorcontrol_config;
            motorcontrol_config.motor_type = BLDC_MOTOR;
            motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
            motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
            motorcontrol_config.hall_offset[0] =  COMMUTATION_OFFSET_CLK;
            motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
            motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

            motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                    c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
        }

7. Run the application (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


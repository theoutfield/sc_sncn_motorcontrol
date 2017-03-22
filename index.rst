.. _somanet_motor_control:

SOMANET Motion Control Library
==============================

The **SOMANET Motion Control Library** provides services and utilities to perform BLDC Motor and Motion Control using SOMANET devices.
The library offers following functionalities:

* BLDC motors control
* Field Oriented Control
* Position, Velocity, and Torque control loops
* Feedback sensor support (Hall Sensor, Incremental Encoder, BiSS, Magnetic Rotary sensors via SPI)
* Basic Motion Profile Generation
* Fully featured ADC drivers
* Configurable software defined PWM module 
* Watchdog
* GPIO server (e.g. for homing-related functions)

.. cssclass:: downloadable-button 

  `Download Library (Develop) <https://github.com/synapticon/sc_sncn_motorcontrol/archive/develop.zip>`_

.. cssclass:: github

  `Visit Public Development Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop>`_

.. _motor_control_hw_compatibility:

Hardware Compatibility
----------------------

.. class:: float-left 

+---------------------------+
| Required SOMANET Hardware |
+===========================+
| 1x SOMANET **Core**       |
+---------------------------+
| 1x SOMANET **IFM**        |
+---------------------------+

.. class:: float-left

+----------------------------------------------------------------------------------------------------------+
| Supported SOMANET Devices                                                                                |
+==========================================================================================================+
| SOMANET Core: :ref:`C22 <core_c22>`, :ref:`C21 DX <core_c21_dx>`                                         |
+----------------------------------------------------------------------------------------------------------+
| SOMANET IFM:  :ref:`DC 100 <ifm_dc100>`, :ref:`DC 300 <ifm_dc300>`, **DC 500**, **DC 1000**, **DC 5000** |
+----------------------------------------------------------------------------------------------------------+

Modules
--------

.. toctree::
	:maxdepth: 1
	:hidden:

	PWM Module <module_pwm/doc/index>
	Position Feedback Module <module_position_feedback/doc/index>
	Hall Sensor Feedback Module <module_hall/doc/index>
	Incremental Encoder Feedback Module <module_qei/doc/index>
	BiSS Encoder Feedback Module <module_biss/doc/index>
	REM 14 Module <module_rem_14/doc/index>
	REM 16MT Module <module_rem_16mt/doc/index>
	Serial Encoder Module <module_serial_encoder/doc/index>
	SPI Master Module <module_spi_master/doc/index>
	ADC Module <module_adc/doc/index>
	Watchdog Module <module_watchdog/doc/index>
	GPIO Server Module <module_gpio/doc/index>
	BLDC Torque Control Library <lib_bldc_torque_control/doc/index>
	Miscellaneous Module <module_misc/doc/index>
	Controllers Implementation Module <module_controllers/doc/index>
	Profile Module <module_profile/doc/index>
	Motion Control Module <module_motion_control/doc/index>
	

This is the complete list of modules currently included in the **SOMANET Motor Control Library**:

* `PWM Module <module_pwm/doc/index.html>`_: Service to generate center-aligned Pulse-Width modulation signals.
* `Position Feedback Module <module_position_feedback/doc/index.html>`_: Service to read position data from various sensors.
* `Hall Sensor Feedback Module <module_hall/doc/index.html>`_: Driver to read the signals from your feedback Hall sensor.
* `Incremental Encoder Feedback Module <module_qei/doc/index.html>`_: Driver to read the signals from your feedback Encoder Interface.
* `BiSS Encoder Feedback Module <module_biss/doc/index.html>`_: Driver to read data from BiSS Encoder.
* `REM 14 Module <module_rem_14/doc/index.html>`_: Driver to read data from REM 14 Encoder.
* `REM 16MT Module <module_rem_16mt/doc/index.html>`_: Driver to read data from REM 16MT Encoder.
* `Serial Encoder Module <module_serial_encoder/doc/index.html>`_: Service to read position data from a Serial Encoder (SPI or BiSS)
* `SPI Master Module <module_spi_master/doc/index.html>`_: Driver to read/write data from/to a SPI Slave.
* `ADC Module <module_adc/doc/index.html>`_: Driver for the ADC on your IFM Drive board.
* `Watchdog Module <module_watchdog/doc/index.html>`_: Driver for the Watchdog on your IFM DC board.
* `GPIO Server Module <module_gpio/doc/index.html>`_: Contains a service to handle the external digital input/outputs of your board.
* `BLDC Torque Control Library <lib_bldc_torque_control/doc/index>`_: Provides a service to control torque of BLDC motors.
* `Miscellaneous Module <module_misc/doc/index.html>`_: Contains constants and utilities used by the library.
* `Controllers Implementation Module <module_controllers/doc/index>`_: Contains PID and NL controllers implementations. 
* `Profile Module <module_profile/doc/index.html>`_: Contains software for profile ramps generation.
* `Motion Control Module <module_motion_control/doc/index>`_: Provides service for position and velocity control.

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	PWM Demo <examples/app_demo_general_pwm/doc/index>
	Watchdog Demo <examples/app_demo_watchdog/doc/index>

	Hall Effect Latched Sensor Test <examples/app_test_hall/doc/index>
	Incremental Encoder Test <examples/app_test_qei/doc/index>
	BiSS Absolute Encoder Test <examples/app_test_biss/doc/index>
	REM 14 Absolute Encoder Test <examples/app_test_rem_14/doc/index.html>
	REM 16MT Absolute Encoder Test <examples/app_test_rem_16mt/doc/index.html>
	Position Feedback Service Test <examples/app_test_position_feedback/doc/index.html>
	
	BLDC Torque Control Demo <examples/app_demo_advanced_foc/doc/index>	
	BLDC Motion Control Demo <examples/app_demo_bldc_position/doc/index>	
	BLDC Control Tuning Demo <examples/app_demo_bldc_velocity_position/doc/index>
	

* **Low level functionality tests:**

	* `PWM Demo <examples/app_demo_general_pwm/doc/index.html>`_: Simple example on PWM signal output.
	* `Watchdog Demo <examples/app_demo_watchdog/doc/index.html>`_: Simple example on how to use the Watchdog.

* **Position Feedback Sensors tests:**

	* `Hall Effect Latched Sensor Test <examples/app_test_hall/doc/index.html>`_: Shows data acquired from a latched Hall sensor served as a rotary encoder.	
	* `Incremental Encoder Test <examples/app_test_qei/doc/index.html>`_: Shows data acquired from an incremental encoder.
	* `BiSS Absolute Encoder Test <examples/app_test_biss/doc/index.html>`_: Shows data acquired from a BiSS interface encoder.
	* `REM 14 Absolute Encoder Test <examples/app_test_rem_14/doc/index.html>`_: Shows data acquired from REM 14 absolute magnetic rotary encoder.
	* `REM 16MT Absolute Encoder Test <examples/app_test_rem_16mt/doc/index.html>`_: Shows data acquired from REM 16MT absolute magnetic rotary encoder.
	* `Position Feedback Service Test <examples/app_test_position_feedback/doc/index.html>`_: Simple example of the Position Feedback Service usage to acquire position from multiple sensors.

* **BLDC Motor Control demos:**

	* `BLDC Torque Control Demo <examples/app_demo_advanced_foc/doc/index>`_: FOC-based torque control of your BLDC motor.
	* `BLDC Motion Control Demo <examples/app_demo_bldc_position/doc/index>`_: Simple example to make Position/Velocity/Torque Control of your BLDC motor.
	* `BLDC Control Tuning Demo <examples/app_demo_bldc_velocity_position/doc/index>`_: Helper application to tune your motion controllers and find commutation angle offset.


Motor Control Quick Guides
---------------------------

.. toctree::
	:hidden:
	:maxdepth: 1

	Motor Control Library configuration <doc/motor_configuration> 
	Commutation Angle Offset Finding Guide <doc/motor_tuning>

In order to learn how to proceed with the configuration of the motors we highly recommend to read the following tutorials:

* `Motor Control Library configuration <doc/motor_configuration.html>`_ : Learn how to configure your Motor Control Library. 
* `Commutation Angle Offset Finding Guide <doc/motor_tuning.html>`_ : Learn how to find your commutating feedback sensor offset for an efficient commutation.

Dependencies
------------

To run your **Motor Control library** it is necessary to include :ref:`SOMANET Base <somanet_base>`.

.. cssclass:: downloadable-button 

  `Download SOMANET Base Library (Develop) <https://github.com/synapticon/sc_somanet-base/archive/develop.zip>`_

.. cssclass:: github

  `Visit SOMANET Base Public Development Repository <https://github.com/synapticon/sc_somanet-base/tree/develop>`_

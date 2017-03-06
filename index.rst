.. _somanet_motor_control:

SOMANET Motor Control Library
==============================

The **SOMANET Motor Control Library** provides services and utilities to perform BLDC Motor Control using SOMANET devices.
The library offers following functionalities:

* BLDC motors control
* Field Oriented Control
* Position, Velocity, and Torque control loops
* Feedback sensor support (Hall Sensor, Incremental Encoder, BiSS, Magnetic Rotary sensors via SPI)
* Basic Motion Profile Generation
* Fully featured ADC driver
* Configurable software defined PWM module 
* Watchdog
* GPIO server (e.g. for homing-related functions)

.. cssclass:: downloadable-button 

  `Download Library <https://github.com/synapticon/sc_sncn_motorcontrol/archive/master.zip>`_

.. cssclass:: github

  `Visit Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/>`_

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
| SOMANET IFM:  :ref:`DC 100 <ifm_dc100>`, :ref:`DC 300 <ifm_dc300>`, **DC 1000**, **DC 5000**             |
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
	BLDC Torque Control Module <module_bldc_torque_control_lib/doc/index>
	Miscellaneous Module <module_misc/doc/index>
	Control Loops Module <module_ctrl_loops/doc/index>
	Profile Module <module_profile/doc/index>
	Controllers Library <module_controllers_lib/doc/index>
	

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
* `BLDC Torque Control Module <module_bldc_torque_control_lib/doc/index>`_: Provides a service to control torque of BLDC motors.
* `Control Loops Module <module_ctrl_loops/doc/index.html>`_: Provide services for position, velocity and control loops. 
* `Profile Module <module_profile/doc/index.html>`_: Contains software for profile ramps generation.
* `Miscellaneous Module <module_misc/doc/index.html>`_: Contains constants and utilities used by the library.
* `Controllers Library <module_controllers_lib/doc/index>`_: Contains PID and NL controllers implementations.

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	Incremental Encoder Feedback Test <examples/app_test_qei/doc/index>
	Hall Sensor Feedback Test <examples/app_test_hall/doc/index>
	BiSS Encoder Feedback Test <examples/app_test_biss/doc/index>
	Watchdog Demo <examples/app_demo_watchdog/doc/index>
	
	PWM Demo <examples/app_demo_general_pwm/doc/index>
	
	BLDC Motor Control Demo <examples/app_demo_bldc_motorcontrol/doc/index>	
	BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index>	
	BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index>
	BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index>
	
	Brushed DC Motor Control Demo <examples/app_demo_brushed_dc/doc/index>
	Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>
	Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>
	Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index>
	Brushed DC Velocity Control with a potentiometer <examples/app_demo_brushed_dc_ext_control/doc/index>

	Commutation Offsets Helper Demo <examples/app_demo_offset_commutation_tuning/doc/index> 

* **Low level functionality tests:**

	* `PWM Demo <examples/app_demo_general_pwm/doc/index.html>`_: Simple example on PWM signal output.
	* `Watchdog Demo <examples/app_demo_watchdog/doc/index.html>`_: Simple example on how to use the Watchdog.

* **Position Feedback Sensors tests:**

	* `Incremental Encoder Interface Test <examples/app_test_qei/doc/index.html>`_: Shows data acquired from incremental Encoder.
	* `Hall Sensor Feedback Test <examples/app_test_hall/doc/index.html>`_: Shows data acquired from Hall sensor.
	* `BiSS Encoder Feedback Test <examples/app_test_biss/doc/index.html>`_: Shows data acquired from BiSS Encoder.
	* `REM 14 Feedback Test <examples/app_test_rem_14/doc/index.html>`_: Shows data acquired from a REM 14 Encoder.
	* `REM 16MT Feedback Test <examples/app_test_rem_16mt/doc/index.html>`_: Shows data acquired from REM 16MT Encoder.
	* `Position Feedback Service Test <examples/app_test_position_feedback/doc/index.html>`_: Simple example of the usage of Position Feedback Service to acquire position from multiple sensors.

* **BLDC Motor Control demos:**

	* `BLDC Motor Control Demo <examples/app_demo_bldc_motorcontrol/doc/index.html>`_: Plain Sinusoidal commutation of your BLDC motor.
	* `BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index.html>`_: Simple example to make Position Control of your BLDC motor.
	* `BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index.html>`_: Simple example to make Velocity Control of your BLDC motor.
	* `BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index.html>`_: Simple example to make Torque Control of your BLDC motor.
	* `Commutation Offsets Helper Demo <examples/app_demo_offset_commutation_tuning/doc/index.html>`_: Example to help you calibrating the commutation offsets of your Hall sensor.

* **Brushed DC Motor Control demos:**

	* `Brushed DC Motor Control Demo <examples/app_demo_brushed_dc/doc/index.html>`_: Simple example to drive your Brushed DC motor.
	* `Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index.html>`_: Simple example to make Position Control of your Brushed DC motor.
	* `Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index.html>`_: Simple example to make Velocity Control of your Brushed DC motor.
	* `Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index.html>`_: Simple example to make Torque Control of your Brushed DC motor.
	* `Brushed DC Control with a potentiometer <examples/app_demo_brushed_dc_ext_control/doc/index.html>`_: Simple example to drive Brushed DC motor by an analog input signal.
	
	

Motor Control Quick Guides
---------------------------

.. toctree::
	:hidden:
	:maxdepth: 1

	Motor Control Library configuration <doc/motor_configuration> 
	Commutation offsets adjustment <doc/motor_tuning>

In order to learn how to proceed with the configuration of the motors we highly recommend to read the following tutorials:

* `Motor Control Library configuration <doc/motor_configuration.html>`_ : Learn how to configure your Motor Control Library. 
* `Commutation offsets adjustment <doc/motor_tuning.html>`_ : Learn how to adjust your feedback Hall sensor offsets for an optimal commutation.

Dependencies
------------

To run your **Motor Control library** it is necessary to include :ref:`SOMANET Base <somanet_base>`.

.. cssclass:: downloadable-button 

  `Download SOMANET Base Library <https://github.com/synapticon/sc_somanet-base/archive/master.zip>`_

.. cssclass:: github

  `Visit SOMANET Base Public Repository <https://github.com/synapticon/sc_somanet-base>`_

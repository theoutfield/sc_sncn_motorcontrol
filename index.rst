.. _somanet_motor_control:

SOMANET Motor Control Library
==============================

The **SOMANET Motor Control Library** provides services and utilities to perform BLDC/BDC Motor Control using SOMANET devices.
The library offer following functionalities:

* BLDC and Brushed DC motors control
* Sinusoidal BLDC commutation
* Position, Velocity and torque control loops (up to 18KHz)
* Feedback sensor support (Hall Sensor, Incremental Encoder, BiSS, AMS Magnetic Rotary sensor via SPI)
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
| SOMANET IFM: **DC30**, :ref:`DC 100 <ifm_dc100>`, :ref:`DC 300 <ifm_dc300>`, **DC 1000**, **DC 5000**    |
+----------------------------------------------------------------------------------------------------------+

Modules
--------

.. toctree::
	:maxdepth: 1
	:hidden:

	Symmetrical PWM Module <module_pwm_symmetrical/doc/index>
	Hall Sensor Feedback Module <module_hall/doc/index>
	Incremental Encoder Feedback Module <module_qei/doc/index>
	BiSS Encoder Feedback Module <module_biss/doc/index>
	ADC Module <module_adc/doc/index>
	Watchdog Module <module_watchdog/doc/index>
	GPIO Server Module <module_gpio/doc/index>
	Motor Control Module <module_motorcontrol/doc/index>
	Miscellaneous Module <module_misc/doc/index>
	Control Loops Module <module_ctrl_loops/doc/index>
	Profile Module <module_profile/doc/index>
	

This is the complete list of modules currently included in the **SOMANET Motor Control Library**:

* `Symmetrical PWM Module <module_pwm_symmetrical/doc/index.html>`_: Service for PWM generation. 
* `Hall Sensor Feedback Module <module_hall/doc/index.html>`_: Driver to read the signals from your feedback Hall sensor.
* `Incremental Encoder Feedback Module <module_qei/doc/index.html>`_: Driver to read the signals from your feedback Encoder Interface.
* `BiSS Encoder Feedback Module <module_biss/doc/index.html>`_: Driver to read data from BiSS Encoder.
* `ADC Module <module_adc/doc/index.html>`_: Driver for the ADC on your IFM DC board.
* `Watchdog Module <module_watchdog/doc/index.html>`_: Driver for the Watchdog on your IFM DC board.
* `GPIO Server Module <module_gpio/doc/index.html>`_: Contains a service to handle the external digital input/outputs of your board.
* `Motor Control Module <module_motorcontrol/doc/index.html>`_: Provide a service to commutate BLDC motors and drive Brushed DC motors.
* `Control Loops Module <module_ctrl_loops/doc/index.html>`_: Provide services for position, velocity and control loops. 
* `Profile Module <module_profile/doc/index.html>`_: Contains software for profile ramps generation.
* `Miscellaneous Module <module_misc/doc/index.html>`_: Contains constants and utilities used by the library.

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	Incremental Encoder Feedback Test <examples/app_test_qei/doc/index>
	Hall Sensor Feedback Test <examples/app_test_hall/doc/index>
	BiSS Encoder Feedback Test <examples/app_test_biss/doc/index>
	Watchdog Demo <examples/app_demo_watchdog/doc/index>
	PWM Symmetrical Demo <examples/app_pwm_symmetrical_demo/doc/index>
	
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

	* `PWM Symmetrical Demo <examples/app_pwm_symmetrical_demo/doc/index.html>`_: Simple example on PWM signal output.
	* `Incremental Encoder Interface Test <examples/app_test_qei/doc/index.html>`_: Shows data acquired from incremental Encoder.
	* `Hall Sensor Feedback Test <examples/app_test_hall/doc/index.html>`_: Shows data acquired from Hall sensor.
	* `BiSS Encoder Feedback Test <examples/app_test_biss/doc/index.html>`_: Shows data acquired from BiSS Encoder 
	* `Watchdog Demo <examples/app_demo_watchdog/doc/index.html>`_: Simple example on how to use the Watchdog.

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

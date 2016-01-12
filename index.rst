.. _somanet_motor_control:

SOMANET Motor Control Library
==============================

The **SOMANET Motor Control Library** gathers services and utilities to perform BLDC/BDC Motor Control using SOMANET devices.
The library is spread over different modules which offer different functionalities:

* BLDC and Brushed DC motors support.
* Sinusoidal commutation.
* PWM Switching frequency up to 18KHz.  
* Hall Effect Feedback sensor support.
* Incremental Encoder Interface support.  
* Fully functional ADC driver.
* Fully functional Watchdog driver.      
* GPIO management support (for homing-related functions).    
* Position, velocity, and torque control loops (up to 18KHz).
* Profiles ramp generation support.

.. cssclass:: downloadable-button 

  `Download Library <https://github.com/synapticon/sc_sncn_motorcontrol/archive/master.zip>`_

.. cssclass:: github

  `Visit Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/>`_

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

+------------------------------------------------------------------------------------------------+
| Supported SOMANET Devices                                                                      |
+================================================================================================+
| SOMANET Core: :ref:`C22 <core_c22>`, :ref:`C21 DX <core_c21_dx>`                               |
+------------------------------------------------------------------------------------------------+
| SOMANET IFM: :ref:`DC 100 <ifm_dc100>`, :ref:`DC 300 <ifm_dc300>`, **DC 1000**, **DC 5000**    |
+------------------------------------------------------------------------------------------------+

Modules
--------

.. toctree::
	:maxdepth: 1
	:hidden:

	Symmetrical PWM Module <module_pwm_symmetrical/doc/index>
	Hall Sensor Module <module_hall/doc/index>	
	Encoder Interface Module <module_qei/doc/index>
	ADC Module <module_adc/doc/index>
	Watchdog Module <module_watchdog/doc/index>
	GPIO Module <module_gpio/doc/index>
	Motorcontrol Module <module_motorcontrol/doc/index>
	Miscellaneous Module <module_misc/doc/index>
	Control Loops Module <module_ctrl_loops/doc/index>
	Profile Module <module_profile/doc/index>
	

This is the complete list of modules currently included in the **SOMANET Motor Control Library**:

* `Symmetrical PWM Module <module_pwm_symmetrical/doc/index>`_ : Service for PWM generation. 
* `Hall Sensor Module <module_hall/doc/index>`_ : Driver to read the signals from your feedback Hall sensor.
* `Encoder Interface Module <module_qei/doc/index>`_ : Driver to read the signals from your feedback Encoder Interface.
* `ADC Module <module_adc/doc/index>`_ : Driver for the ADC on your IFM DC board.
* `Watchdog Module <module_watchdog/doc/index>`_ : Driver for the Watchdog on your IFM DC board.
* `GPIO Module <module_gpio/doc/index>`_ : Contains a service to handle the external digital input/outputs of your board.
* `Motor Control Module <module_motorcontrol/doc/index>`_ : Provide a service to commutate BLDC motors and drive Brushed DC motors.
* `Control Loops Module <module_ctrl_loops/doc/index>`_ : Provide services for position, velocity and control loops. 
* `Profile Module <module_profile/doc/index>`_ : Contains software for profile ramps generation.
* `Miscellaneous Module <module_misc/doc/index>`_ : Contains constants and utilities used by the library.

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	Encoder Interface Test <examples/app_test_qei/doc/index>
	Hall Sensor Test <examples/app_test_hall/doc/index>
	Watchdog Demo <examples/app_demo_watchdog/doc/index>
	PWM Symmetrical Demo <examples/app_pwm_symmetrical_demo/doc/index>
	
	BLDC Motor Control Demo <examples/app_demo_motorcontrol/doc/index>	
	BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index>	
	BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index>
	BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index>
	
	Brushed DC Motor Control Demo <examples/app_demo_brushed_dc/doc/index>
	Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>
	Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>
	Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index>

	Brushed DC Velocity Control over analog Input <examples/app_demo_brushed_dc_ext_regulated/doc/index>
	Commutation Offsets Helper Demo <examples/app_demo_offset_commutation_tuning/doc/index> 
	
	Slave Standalone Position Control Demo <examples/app_demo_bldc_position/doc_quickstart/quickstart>
	Slave Standalone Velocity Control Demo <examples/app_demo_bldc_velocity/doc_quickstart/quickstart>
	Slave Standalone Torque Control Demo <examples/app_demo_bldc_torque/doc_quickstart/quickstart>

* **Low level functionality tests:**

	* `PWM Symmetrical Demo <examples/app_pwm_symmetrical_demo/doc/index>`_: Simple example on PWM signal output.
	* `Encoder Interface Test <examples/app_test_qei/doc/index>`_: Show the data read from your feedback Encoder Interface.
	* `Hall Sensor Test <examples/app_test_hall/doc/index>`_: Show the data read from your feedback Hall sensor.
	* `Watchdog Demo <examples/app_demo_watchdog/doc/index>`_: Simple example on how to use the Watchdog.

* **BLDC Motorcontrol demos:**

	* `BLDC Motor Control Demo <examples/app_demo_motorcontrol/doc/index>`_: Plain Sinusoidal commutation of your BLDC motor.
	* `BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index>`_: Simple example to make Position Control of your BLDC motor.
	* `BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index>`_: Simple example to make Velocity Control of your BLDC motor.
	* `BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index>`_: Simple example to make Torque Control of your BLDC motor.
	* `Commutation Offsets Helper Demo <examples/app_demo_offset_commutation_tuning/doc/index>`_: Example to help you calibrating the commutation offsets of your Hall sensor.

* **Brushed DC Motorcontrol demos:**

	* `Brushed DC Motor Control Demo <examples/app_demo_brushed_dc/doc/index>`_: Plain commutation of your BLDC motor.
	* `Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>`_: Simple example to make Position Control of your Brushed DC motor.
	* `Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>`_: Simple example to make Velocity Control of your Brushed DC motor.
	* `Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index>`_: Simple example to make Torque Control of your Brushed DC motor.
	* `Brushed DC Velocity Control over analog Input <examples/app_demo_brushed_dc_ext_regulated/doc/index>`_: Example to control the speed of your Brushed DC motor over an analog input.

Motor Control Quick Guides
---------------------------

.. toctree::
	:hidden:
	:maxdepth: 1

	Motor Control Library configuration <doc/motor_configuration> 
	Commutation offsets adjustment <doc/motor_tuning>

In order to learn how to proceed with the configuration of the motors we highly recommend to read the following tutorials:

* `Motor Control Library configuration <doc/motor_configuration>`_ : Learn how to configure your Motor Control Library. 
* `Commutation offsets adjustment <doc/motor_tuning>`_ : Learn how to adjust your feedback Hall sensor offsets for an optimal commutation.

Dependencies
------------

To run your **Motorcontrol library** it is neccesary to include :ref:`SOMANET Base <somanet_base>`.

.. cssclass:: downloadable-button 

  `Download SOMANET Base Library <https://github.com/synapticon/sc_somanet-base/archive/master.zip>`_

.. cssclass:: github

  `Visit SOMANET Base Public Repository <https://github.com/synapticon/sc_somanet-base>`_

SOMANET Motor Control Library
==============================

.. toctree::
	:maxdepth: 1
	:hidden:

	Symmetrical PWM Module <module_pwm_symmetrical/doc/index>	
	Motorcontrol Module <module_motorcontrol/doc/index>
	Hall Sensor Module <module_hall/doc/index>
	Encoder Interface Module <module_qei/doc/index>
	ADC Module <module_adc/doc/index>
	Blocks Module <module_blocks/doc/index>
	Control Loops Module <module_ctrl_loops/doc/index>
	GPIO Module <module_gpio/doc/index>
	Motorcontrol Common Module <module_motorcontrol_common/doc/index>
	Profile Module <module_profile/doc/index>
	Watchdog Module <module_watchdog/doc/index>
	Symmetrical PWM Module <module_pwm_symmetrical/doc/index>

The **SOMANET Motor Control Library** provides Motor Control support for SOMANET devices. 

Our SOMANET Motor Control Library is composed of the following modules:

* `Profile Module <module_profile/doc/index>`_ : Lorem ipsum... 
* `Control Loops Module <module_ctrl_loops/doc/index>`_ : Lorem ipsum... 
* `Motorcontrol Module <module_motorcontrol/doc/index>`_ : Lorem ipsum...
* `Motorcontrol Common Module <module_motorcontrol_common/doc/index>`_ : Lorem ipsum... 
* `Blocks Module <module_blocks/doc/index>`_ : Lorem ipsum...
* `ADC Module <module_adc/doc/index>`_ : Lorem ipsum...
* `GPIO Module <module_gpio/doc/index>`_ : Lorem ipsum... 
* `Watchdog Module <module_watchdog/doc/index>`_ : Lorem ipsum... 
* `Hall Sensor Module <module_hall/doc/index>`_ : Lorem ipsum...
* `Encoder Interface Module <module_qei/doc/index>`_ : Lorem ipsum...
* `Symmetrical PWM Module <module_pwm_symmetrical/doc/index>`_ : Lorem ipsum...

.. cssclass:: downloadable-button 

  `Download Library <https://github.com/synapticon/sc_sncn_motorcontrol/archive/new_board_support_system.zip>`_

.. cssclass:: github

  `Visit Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/new_board_support_system>`_

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	Encoder Interface Test <examples/app_test_qei/doc/index>
	Hall Sensor Test <examples/app_test_hall/doc/index>
	Watchdog Demo <examples/app_watchdog_demo/doc/index>
	PWM Symmetrical Demo <examples/app_pwm_symmetrical_demo/doc/index>
	
	Motorcontrol Demo <examples/app_demo_motorcontrol/doc/index>	
	BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index>	
	BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index>
	BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index>
	
	Brushed DC Motorcontrol Demo <examples/app_demo_brushed_dc/doc/index>
	Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>
	Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>
	Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index>

	Brushed DC Velocity Control over analog Input <examples/app_demo_brushed_dc_ext_regulated/doc/index>
	Commutation Offsets Helper Demo <examples/app_demo_offset_commutation_tuning/doc/index> 
	
	Slave Standalone Position Control Demo <examples/app_demo_bldc_position/doc_quickstart/quickstart>
	Slave Standalone Velocity Control Demo <examples/app_demo_bldc_velocity/doc_quickstart/quickstart>
	Slave Standalone Torque Control Demo <examples/app_demo_bldc_torque/doc_quickstart/quickstart>


* `Encoder Interface Test <examples/app_test_qei/doc/index>`_: Show the data read from your feedback Encoder Interface.
* `Hall Sensor Test <examples/app_test_hall/doc/index>`_: Show the data read from your feedback Hall sensor.
* `Watchdog Demo <examples/app_watchdog_demo/doc/index>`_: Simple example on how to use the Watchdog.
* `PWM Symmetrical Demo <examples/app_watchdog_demo/doc/index>`_: Simple example on PWM signal output.

* `Motorcontrol Demo <examples/app_demo_motorcontrol/doc/index>`_: Plain Sinusoidal commutation of your BLDC motor.
* `BLDC Position Control Demo <examples/app_demo_bldc_position/doc/index>`_: Simple example to make Position Control of your BLDC motor.
* `BLDC Velocity Control Demo <examples/app_demo_bldc_velocity/doc/index>`_: Simple example to make Velocity Control of your BLDC motor.
* `BLDC Torque Control Demo <examples/app_demo_bldc_torque/doc/index>`_: Simple example to make Torque Control of your BLDC motor.

* `Brushed DC Motorcontrol Demo <examples/app_demo_brushed_dc/doc/index>`_: Plain commutation of your BLDC motor.
* `Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>`_: Simple example to make Position Control of your Brushed DC motor.
* `Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>`_: Simple example to make Velocity Control of your Brushed DC motor.
* `Brushed DC Torque Control Demo <examples/app_demo_brushed_dc_torque/doc/index>`_: Simple example to make Torque Control of your Brushed DC motor.

* `Brushed DC Position Control Demo <examples/app_demo_brushed_dc_position/doc/index>`_: Simple example to make Position Control of your Brushed DC motor.
* `Brushed DC Velocity Control Demo <examples/app_demo_brushed_dc_velocity/doc/index>`_: Simple example to make Velocity Control of your Brushed DC motor.

Drive Configuration
--------------------

.. toctree::
	:hidden:
	:maxdepth: 1

	Drive Configuration <howto/motor_configuration> 
	Drive Tuning <howto/motor_tuning>

In order to learn how to proceed with the configuration of the motors we highly recommend to read the following tutorials:

* `Drive Configuration <howto/motor_configuration>`_ : Lorem ipsum... 
* `Drive Tuning <howto/motor_tuning>`_ : Lorem ipsum... 

Dependencies
------------

To run your **Motorcontrol library** it is neccesary to include **SOMANET Base Library**.

.. cssclass:: downloadable-button 

  `Download SOMANET Base Library <https://github.com/synapticon/sc_somanet-base/archive/master.zip>`_

.. cssclass:: github

  `Visit SOMANET Base Public Repository <https://github.com/synapticon/sc_somanet-base>`_

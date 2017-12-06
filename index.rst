.. _somanet_motion_control:

SOMANET Motion Control Component
================================

The **SOMANET Motion Control Component** provides modules and libraries to perform BLDC Motor and Motion Controls using SOMANET devices.
The component offers the following functionalities:

* BLDC motors control
* Field Oriented Control (FOC)
* Position, Velocity, and Torque control loops
* Runtime reconfigurable Feedback Sensor support (Hall Sensor, Incremental (AB and ABI), BiSS, and Magnetic Rotary encoders via SPI)
* Basic Motion Profile Generation
* Fully featured ADC drivers
* Configurable software-defined PWM module 
* Watchdog
* GPIO server (e.g. for homing-related functions)

.. cssclass:: downloadable-button 

  `Download Component <https://github.com/synapticon/sc_sncn_motorcontrol/archive/master.zip>`_

.. cssclass:: github

  `Visit Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master>`_

.. _motion_control_hw_compatibility:

Hardware Compatibility
----------------------

.. class:: float-left 

+---------------------------+
| Required SOMANET Hardware |
+===========================+
| 1x SOMANET **Core**       |
+---------------------------+
| 1x SOMANET **Drive**      |
+---------------------------+

.. class:: float-left

+----------------------------------------------------------------------------------------------------------+
| Supported SOMANET Devices                                                                                |
+==========================================================================================================+
| SOMANET Core: :ref:`C22 <core_c22>`, :ref:`C21 DX <core_c21_dx>`                                         |
+----------------------------------------------------------------------------------------------------------+
| SOMANET Drive:  :ref:`Drive 100 <ifm_dc100>`, :ref:`Drive 1000 <ifm_dc1000_b2>`                          |
+----------------------------------------------------------------------------------------------------------+

Modules
--------

.. toctree::
	:maxdepth: 1
	:hidden:

	PWM Module <module_pwm/doc/index>
	Position Feedback Module <module_position_feedback/doc/index>
	Hall Sensor Feedback Module <module_hall_sensor/doc/index>
	Incremental Encoder Feedback Module <module_incremental_encoder/doc/index>
	BiSS / SSI Encoder Feedback Module <module_biss_encoder/doc/index>
	REM 14 Module <module_encoder_rem_14/doc/index>
	REM 16MT Module <module_encoder_rem_16mt/doc/index>
	Serial Encoder Module <module_serial_encoder/doc/index>
	SPI Master Module <module_spi_master/doc/index>
	ADC Module <module_adc/doc/index>
	Watchdog Module <module_watchdog/doc/index>
	GPIO Server Module <module_gpio/doc/index>
	BLDC Torque Control Library <lib_bldc_torque_control/doc/index>
	Miscellaneous Module <module_utils/doc/index>
	Controllers Implementation Module <module_controllers/doc/index>
	Profile Module <module_profiles/doc/index>
	Motion Control Module <module_motion_control/doc/index>
	Shared Memory Module <module_shared_memory/doc/index>
	

This is the complete list of modules and libraries currently included into the **SOMANET Motion Control Component**:

* `PWM Module <module_pwm/doc/index.html>`_: Service to generate center-aligned Pulse-Width modulation signals.
* `Position Feedback Module <module_position_feedback/doc/index.html>`_: Service to read position data from various sensors.
* `Hall Sensor Feedback Module <module_hall_sensor/doc/index.html>`_: Driver to read signals from a latched Hall-effect based feedback sensor.
* `Incremental Encoder Feedback Module <module_incremental_encoder/doc/index.html>`_: Driver to read signals from an Incremental (AB/ABI) Encoder.
* `BiSS / SSI Encoder Feedback Module <module_biss_encoder/doc/index.html>`_: Driver to read data from a BiSS-C or SSI inteface Encoder.
* `REM 14 Encoder Module <module_encoder_rem_14/doc/index.html>`_: Driver to read data from a SYNAPTICON featured REM 14 Encoder.
* `REM 16MT Encoder Module <module_encoder_rem_16mt/doc/index.html>`_: Driver to read data from a SYNAPTICON featured REM 16MT Encoder.
* `Serial Encoder Module <module_serial_encoder/doc/index.html>`_: Combined service to read position feedback data from Serial Encoders (SPI or BiSS)
* `SPI Master Module <module_spi_master/doc/index.html>`_: Driver to read/write data from/to an SPI slave device.
* `ADC Module <module_adc/doc/index.html>`_: Driver for the ADC of your Drive board.
* `Watchdog Module <module_watchdog/doc/index.html>`_: Driver for the Watchdog on your Drive board.
* `GPIO Server Module <module_gpio/doc/index.html>`_: Provides a service to handle the external digital input/outputs of your Drive board.
* `BLDC Torque Control Library <lib_bldc_torque_control/doc/index.html>`_: Provides a service to control torque of BLDC motors.
* `Miscellaneous Module <module_utils/doc/index.html>`_: Contains constants and utilities used by the library.
* `Controllers Implementation Module <module_controllers/doc/index.html>`_: Contains PID and NL controllers implementations. 
* `Profile Module <module_profiles/doc/index.html>`_: Contains software for profile ramps generation.
* `Motion Control Module <module_motion_control/doc/index.html>`_: Provides service for position and velocity control.
* `Shared Memory Module <module_shared_memory/doc/index.html>`_: Enables asynchronous data access among tasks

Examples
--------

.. toctree::
	:hidden:
	:maxdepth: 1

	PWM Demo <examples/app_demo_general_pwm/doc/index>
	Watchdog Demo <examples/app_demo_watchdog/doc/index>

	Hall Sensor Test <examples/app_test_hall_sensor/doc/index>
	Incremental Encoder Test <examples/app_test_incremental_encoder/doc/index>
	BiSS Absolute Encoder Test <examples/app_test_biss_encoder/doc/index>
	SSI Encoder Test <examples/app_test_ssi_encoder/doc/index>
	REM 14 Absolute Encoder Test <examples/app_test_rem_14_encoder/doc/index>
	REM 16MT Absolute Encoder Test <examples/app_test_rem_16mt_encoder/doc/index>
	Position Feedback Service Test <examples/app_test_position_feedback/doc/index>
	
	BLDC Control Tuning Demo <examples/app_control_tuning/doc/index>	
	BLDC Torque Control Demo <examples/app_demo_advanced_foc/doc/index>	
	BLDC Motion Control Demo <examples/app_demo_motion_control/doc/index>	
	

* **Low level functionality tests:**

	* `PWM Demo <examples/app_demo_general_pwm/doc/index.html>`_: Simple example on PWM signal output.

* **Position Feedback Sensors tests:**

	* `Hall Sensor Test <examples/app_test_hall_sensor/doc/index.html>`_: Shows data acquired from a latched Hall sensor served as a rotary encoder.	
	* `Incremental Encoder Test <examples/app_test_incremental_encoder/doc/index.html>`_: Shows data acquired from an incremental encoder.
	* `BiSS Absolute Encoder Test <examples/app_test_biss_encoder/doc/index.html>`_: Shows data acquired from a BiSS interface encoder.
	* `SSI Encoder Test <examples/app_test_ssi_encoder/doc/index.html>`_: Shows data acquired from a SSI interface encoder.
	* `REM 14 Absolute Encoder Test <examples/app_test_rem_14_encoder/doc/index.html>`_: Shows data acquired from REM 14 absolute magnetic rotary encoder.
	* `REM 16MT Absolute Encoder Test <examples/app_test_rem_16mt_encoder/doc/index.html>`_: Shows data acquired from REM 16MT absolute magnetic rotary encoder.
	* `Position Feedback Service Test <examples/app_test_position_feedback/doc/index.html>`_: Simple example of the Position Feedback Service usage to acquire position from multiple sensors.

* **BLDC Motor Control and Tuning demos:**

	* `BLDC Control Tuning Demo <examples/app_control_tuning/doc/index.html>`_: Helper application to tune motion controllers and find commutation angle offset.
	* `BLDC Torque Control Demo <examples/app_demo_torque_control/doc/index.html>`_: FOC-based torque control of BLDC motors.
	* `BLDC Motion Control Demo <examples/app_demo_motion_control/doc/index.html>`_: Simple example to make Position/Velocity/Torque Control of BLDC motors.


Motor Control Quick Guides
---------------------------

.. toctree::
	:hidden:
	:maxdepth: 1

	Motion Control Library configuration <doc/motor_configuration> 

In order to learn how to proceed with the configuration of the motors we highly recommend to read the following tutorials:

* `Motion Control Components configuration <doc/motor_configuration.html>`_ : Learn how to configure your Motion Control Components. 

Dependencies
------------

To run your **Motor Control library** it is necessary to include :ref:`SOMANET Base <somanet_base>`.

.. cssclass:: downloadable-button 

  `Download SOMANET Base Library <https://github.com/synapticon/sc_somanet-base/archive/master.zip>`_

.. cssclass:: github

  `Visit SOMANET Base Public Repository <https://github.com/synapticon/sc_somanet-base/tree/master>`_

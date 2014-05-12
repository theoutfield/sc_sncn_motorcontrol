Sinusoidal Motor Control SOFTWARE COMPONENT 
===============
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

Implementation of Sinusoidal Motor Control for BLDC drives on SOMANET devices.

* [How to?](https://github.com/synapticon/sc_sncn_motorctrl/tree/master/howto)

<table>
<tr>
  <td width="150px" height="30px">Latest release: </td>
  <td width="300px"><a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/releases/tag/v1.0">1.0</a></td>
</tr>
<tr>
  <td height="30px">Maintainer:</td>
  <td><a href="mailto:support@synapticon.com">support@synapticon.com</a></td>
</tr>
</table> 

Key Features
---------
   * Sinusoidal Commutation
   * Profile Position Control 
   * Profile Velocity Control
   * Profile Torque Control
   * Homing feature
   * Ethercat Operating Modes
   * Support QEI sensor with Index/ no Index
   * Support Hall sensor
   * Support Analog sensor 
   * Support GPIO Digital
   * Precise position control based on position sensor ticks

Components
---------

| Module        				| Demo          						|
| :-------------: 				|:-------------						|
| [module_adc][module_adc]      		| [test_adc_external_input][test_adc_external_input] 		|
| [module_blocks][module_blocks] 		|       							|
| [module_comm][module_comm]	 		|     								|
| [module_common][module_common]		|     								|
| [module_commutation][module_commutation]	|								|
| [module_ctrl_loops][module_ctrl_loops]	| [test_position-ctrl][test_position-ctrl] [test_velocity-ctrl][test_velocity-ctrl]	|
| [module_ecat_drive][module_ecat_drive]	| [test_ethercat-motorctrl-mode][test_ethercat-motorctrl-mode]	|
| [module_gpio][module_gpio]			| [test_gpio_digital][test_gpio_digital] [test_homing][test_homing] 	|
| [module_hall][module_hall]			| [test_hall][test_hall]					|
| [module_profile][module_profile]		|								|
| [module_qei][module_qei]			| [test_qei][test_qei]						|
| [module_sm][module_sm]			|								|


Required software (dependencies)
---------
  * [sc_somanet-base](https://github.com/synapticon/sc_somanet-base) 
  * [sc_pwm](https://github.com/synapticon/sc_pwm)
  * [sc_sncn_ethercat](https://github.com/synapticon/sc_sncn_ethercat) (only if using Ethercat Operating Modes)

Changelog
---------
  * [1.0](https://github.com/synapticon/sc_sncn_motorctrl_sin/releases/tag/v1.0) (2014-04-17)
	* Support GPIO Digital ports
	* Homing feature
	* Precise Position Control based on position sensor ticks
  * [0.9beta](https://github.com/synapticon/sc_sncn_ctrlproto/releases/tag/v0.9-beta) (2013-01-24)

License
---------

Please see [LICENSE](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/LICENSE.md).


[sc_sncn_ethercat]:https://github.com/synapticon/sc_sncn_ethercat
[sc_pwm]: https://github.com/synapticon/sc_pwm
[sc_somanet-base]: https://github.com/synapticon/sc_somanet-base

[module_adc]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_adc
[module_hall]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall
[module_watchdog]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_watchdog
[module_ecat_drive]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ecat_drive
[module_ctrl_loops]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ctrl_loops
[module_blocks]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_blocks
[module_qei]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei
[module_commutation]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_commutation
[module_gpio]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio
[module_common]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_common
[module_sm]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_sm
[module_homing]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_homing
[module_profile]:https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_profile
[module_comm]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_comm

[test_adc_external_input]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ecat_drive
[test_position-ctrl]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_position-ctrl
[test_velocity-ctrl]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_velocity-ctrl
[test_gpio_digital]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_gpio_digital
[test_hall]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_hall
[test_qei]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_qei
[test_homing]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_homing
[test_ethercat-motorctrl-mode]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_ethercat-motorctrl-mode

[module_ethercat]: https://github.com/synapticon/sc_sncn_ethercat/tree/master/module_ethercat

[module_pwm_symmetrical]: https://github.com/synapticon/sc_pwm/tree/master/module_pwm_symmetrical

[module_nodeconfig]: https://github.com/synapticon/sc_somanet-base/tree/master/module_nodeconfig

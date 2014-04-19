Sinusoidal Motor Control SOFTWARE COMPONENT 
===============
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
<br/>
<br/>

Implementation of Sinusoidal Motor Control for BLDC drives on SOMANET devices.

  * [DOCUMENTATION](http://synapticon.github.io/sc_sncn_motorctrl_sin/)

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

Known Issues
---------
  * None

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

Please see [LICENSE](http://synapticon.github.io/sc_sncn_motorctrl_sin/legal.html).

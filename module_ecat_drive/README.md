EtherCAT Drive Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides a communication bridge between Ethercat and Motor drive System; 
complex motor drive/control functionalities. The implemtation receives/sends data to 
the Ethercat Master Application; monitors the state of the motor drive, sensor drive 
and control servers; runs State Machine; initiates, starts and executes/shutsdown a 
particular operation requested from the Ethercat Master Application; packs the sensor
data information to be sent to Ethercat master application.  

To include this module add module_ecat_drive to USED_MODULES in the application/test
makefile, and include header files: ecat_motor_drive.h


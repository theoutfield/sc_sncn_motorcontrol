Blocks Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides watchdog server, filter implementation and drive utilities. 
The watchdog server monitors CORE 3 and disables motor phases in case of emergency.
A moving average filter implementation is provided under filter_blockd for sensor
data filtering. The util provides the respective lookup tables for sinusoidal 
commutation of the motor and motor phase current estimation.

To include this module add module_blocks to USED_MODULES in the application/test
makefile, and include header file watchdog.h, filter_blocks.h and sine_table_big.h

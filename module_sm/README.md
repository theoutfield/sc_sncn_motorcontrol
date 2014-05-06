State Machine Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provide State machine implementation for controllling power drive system and
contains definitions for Controlword, Statusword acc. to ETG.6010 CiA402 Implementation 
Directives; functions to check initialization state of the interface module drivers and 
control servers.

To include this module add module_sm to USED_MODULES in the application/test makefile, 
and include header file drive_config.h


Communication Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>
This module provides functions to read/write data to ctrlproto data structure which
inturn is passed to ethercat Master applications; functions to read SDO (software 
defined objects used for motor/sensor configuration); functions to update respective 
Servers running on interface module (IFM_CORE).

To include this module add module_comm to USED_MODULES in the application/test
makefile, and include header files: comm.h


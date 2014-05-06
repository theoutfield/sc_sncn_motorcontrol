General Purpose I/O Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides driver for the GPIO digital ports on the interface module (IFM).
The module provides GPIO server thread which configures the GPIO ports; read/write GPIO 
digital ports in a while loop; and provides client functions to configure ports and 
read/write ports. 

To include this module add module_gpio to USED_MODULES in the application/test
makefile, and include header files: gpio_server.h and gpio_client.h

Note: For C22 core module this server must be run on CORE 3 and for C21 core module on 
CORE 1, since only these cores have physical connection to the IFM modules.

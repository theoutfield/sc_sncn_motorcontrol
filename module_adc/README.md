ADC Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides driver for the ADCs on the interface module (IFM).
The module provides ADC server thread which drives the ADC and acquires analog 
input data in a while loop; and provides client functions to get data from the 
ADC server.

To include this module add module_adc to USED_MODULES in the application/test
makefile, and include header files: adc_server_ad7949.h and adc_client_ad7949.h

Note: For C22 core module this server must be run on CORE 3 and for C21 core module on 
CORE 1, since only these cores have physical connection to the IFM modules.



\page mods
\section module_adc

The module provides ADC server which drives the ADC included within the IFM Drive DC 100/300 boards. 
The server acquires analog input data in a loop.

The module also  and provides client functions to obtain data from the existing ADC server.

TILE constrains: IFM* (need access to IFM ports)

Examples:

* test_adc_external_input.xc - Documentation \ref blabla



In order to access this module from your application:

1. Add module_adc in your app makefile:

USED_MODULES = module_adc

2. Include the header files on your code:

* adc_server_ad7949.h - To access the server side functions.

* adc_client_ad7949.h - To access the client side functions.


To get started with SOMANET [come here] (website)!


Note: For Core C22, IFM tile is located on CORE 3. For Core C21, IFM tile is on 
CORE 1.

*/

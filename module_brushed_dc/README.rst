SOMANET Brushed-DC Driver Component
===================================

:scope: General Use
:description: SOMANET Brushed-DC Driver
:keywords: SOMANET, BDC, Brushed, DC, Driver
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300

Description
-----------

This component provides a Brushed DC Motor driver for SOMANET IFM-Drive hardware modules. The component consists of motor drive which internally makes use of the pre-driver to drive FETs and configurations under PWM. Further it provides client functions to set input voltage for the brushed DC motor, get fet_state from the Drive Server.

To include this component add module_brushed_dc to USED_MODULES in the application/test makefile, and include header files: brushed_dc_client.h and brushed_dc_server.h 

Note: For SOMANET Core-C22 modules this server must be run on TILE 3 and for SOMANET Core-C21 modules on TILE 1, since only these TILEs have physical connection to the IFM modules.

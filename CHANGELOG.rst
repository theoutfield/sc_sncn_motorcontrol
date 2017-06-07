sc_sncn_motorcontrol Change Log
===============================

3.1
---

  Known Issues:
    * REM 16MT sensor does not work with Core C21 and DC1K with long cables (more than 15 cm).

    This is probably caused by differences in traces length between the GPIO ports used for SPI. A workaround is to flip the Clock (GPIO 1) and the MISO (GPIO 2) pins. For this you need to modify or make a new cable with the two wires flipped
    and also flip the definitions of GPIO ports 1 and 2 in the IFM module's bsp file.
      


3.0.2
-----

  * Rename MOTOR_PHASES_CONFIGURATION in user_config.h and main.xc files.

3.0.1
-----

  * Fix issue with a not working Debug/Release build configuration for demo apps
  * Fix app_demo_torque_control to accept negative torque reference from console
  * Update lib_bldc_torque_control to work with xTIMEcomposer v.14.3.0

3.0.0
-----

  * Channels are completeley replaced by interfaces
  * New configurable PWM module
  * New flexible software architecture
  * New runtime configurable feedback service 
  * Support of any two feedback sensors including two absolute encoders
  * Fully featured BiSS-C interface support
  * Support for two new Absolute Magnetic Rotary encoders offered by SYNAPTICON
  * New model-predictive torque control
  * Removed support of sine commutation 
  * Removed support of Brushed DC motors
  * Supports new XS2 (xCORE-200) architecture  
  * Added built-in profiles for position/velocity/torque control
  * Added new Nonlinear position controll still supporting classical PID options
 
2.0.0
-----

  * Library consistently uses interfaces instead of channels (except PWM :) )
  * Services now configured exclusively trough a structure passed as an argument, no more global defines.
  * Most services now [combinable], so running multiple instances of e.g. service_motorcontrol (former sin_commutation) on a single core is possible
  * Renamed services with "*_service" pattern e.g.run_hall -> hall_service
  * Uses new board support system sc_somanet-base 3.0
  * Added support for BiSS encoders
  * Added many new cool demo apps
  * Removed dependency on external libs sc_sncn_ethercat and sc_pwm
  * New beautiful documentation

1.0.3
-----

  * Switch to sc_somanet-base 2.0.0

1.0.2
------

  * Rename module_sm to module_statemachine
  * Fixed torque calculation initialization 
  * Update diagrams for modules commutation, hall, and qei 

1.0.1
-----

  * Remove initialization functions from commutation_server, hall_server and qei_server API (now executed internally)
  * Resolve unnecessary dependencies
  * Reduced code size and readability
  * Improve documentation

1.0.0
-----

  * Support GPIO ports
  * Homing feature
  * Precise Position Control based on position sensor ticks
  
0.9.0
-----

  * Initial Version




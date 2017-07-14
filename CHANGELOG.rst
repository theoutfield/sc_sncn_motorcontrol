sc_sncn_motorcontrol Change Log
===============================

3.1
---
  * Add automatic PID tuning for cascaded and limitted torque position controllers
  * Add automatic velocity tuning
  * Add cogging compensation
  * Add low-pass filter into position control
  * Add SSI position sensor support
  * Add dual BiSS position sensor support (only DC1K-rev.D1 compatible)
  * Add error detection for open phase
  * Add automatic position sensor eveluation on offset tuning 


  Known Issues:
    * REM 16MT sensor does not work with Core C21-DX_G2 and DC1K rev.C3-C4 with long cables (more than 15 cm).

    This is caused by differences in traces length between the GPIO ports used for SPI. A workaround is to flip the Clock (GPIO 1) and the MISO (GPIO 2) pins. For this you need to modify or make a new cable with the two wires flipped
    and also flip the definitions of GPIO ports 1 and 2 in the IFM module's bsp file.


3.0.3
-----

  * Fix value of Encoder Number of Channels to 2 for AB and 3 for ABI.
  * Fix Hall service position initialization.
  * Fix integral issue in PID controllers (Reset integral value in case new ki is 0).
  * Do not change the integral limits of position/velocity controllers in case automatic tuners are called

3.0.2
-----

  * Rename MOTOR_PHASES_CONFIGURATION in user_config.h and main.xc files.
  * Fix position control strategy values: POS_PID_CONTROLLER = 1, POS_PID_VELOCITY_CASCADED_CONTROLLER = 2, NL_POSITION_CONTROLLER = 3, VELOCITY_PID_CONTROLLER = 4
  * Fix bug in gpio service:
  * only use gpio on the first position_feedback_service
  * fix gpio ports config inside position_feedback_service
  * disable gpio port when used by BiSS
  * disable gpio service in motorcontrol demo apps
  * Fix BiSS multiturn with inverted sensor polarity
  * Fix sensor polarity in sensor test apps


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




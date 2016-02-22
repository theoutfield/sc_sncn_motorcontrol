sc_sncn_motorcontrol Change Log
===============================

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




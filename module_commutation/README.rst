SOMANET BLDC Commutation Component
==================================

:scope: General Use
:description: SOMANET Motor Control Commutation module
:keywords: SOMANET, BLDC, motor, driver, sinusoidal, commutation
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300

Description
-----------

The BLDC Commutation Component provides a driver for BLDC motors, featuring an efficient sinusoidal commutation method. The module implements a Commutation-Server which requires motor angle information provided by a Hall effect sensor (see HALL server in module_hall) and commutates the motor. This component can be easily interfaced and used as a foundation for a custom motor control application. The component can be configured with several parameters like: commutation offset, motor winding type, nominal motor speed and number of pole pairs.

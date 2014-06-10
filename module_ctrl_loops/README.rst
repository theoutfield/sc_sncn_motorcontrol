SOMANET Motor Control loops
===========================

:scope: General Use
:description: Control loops for position, velocity, and torque control
:keywords: SOMANET
:boards: SOMANET IFM-DC100, SOMANET IFM-DC300


Description
-----------

This module provides three control loops for position, velocity and
torque control. Each of these control modes has separate Server loop and
Client functions. The control loops implemented are closed on data from
sensors like position from hall sensor/qei sensor for Position/Velocity
Controller; torque from current sensors for Torque Controller.

The Client provides functions to check status of the controller; update
sensor related parameters needed by the controller; enable/shutdown
controllers; set target values like position/velocity/torque for
controllers; get actual values like position/velocity/torque.

To include this module add module\_ctrl\_loops to USED\_MODULES in the
application/test makefile, and include the following header files for:
Velocity Control: velocity\_ctrl\_client.h and velocity\_ctrl\_server.h
Torque Control: torque\_ctrl\_client.h and torque\_ctrl\_server.h
Position Control: position\_ctrl\_client.h and position\_ctrl\_server.h

Note: The controllers are not limited to only the sensors listed, for
example if you have a different type of position sensor and is
interfaced to the IFM, all it needs is a specific sensor driver running
on IFM Core and respective client functions to read sensor information
must be placed in controller implementation.

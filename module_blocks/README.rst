Blocks Module
=============

:scope: General Use
:description: SOMANET Motor Control utility functions
:keywords: SOMANET
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300


Key Features
------------

  * Watchdog
  * Filter functions
  * Sine lookup tables

Description
-----------

This module provides watchdog server, filter implementation and drive
utilities.

The watchdog server disables the motor phases in case of emergency. A
moving average filter implementation is provided under filter\_blockd
for sensor data filtering.

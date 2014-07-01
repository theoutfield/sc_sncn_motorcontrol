SOMANET EtherCAT Drive Module
=============================

:scope: General Use
:description: EtherCAT communication bridge for Motor Control
:keywords: SOMANET
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300


Description
-----------

This module provides a communication bridge between EtherCAT and Motor
drive System; complex motor drive/control functionalities. The
implemtation receives/sends data to the EtherCAT Master Application;
monitors the state of the motor drive, sensor drive and control servers;
runs State Machine; initiates, starts and executes/shutsdown a
particular operation requested from the EtherCAT Master Application;
packs the sensor data information to be sent to EtherCAT master
application.

To include this module add module\_ecat\_drive to USED\_MODULES in the
application/test makefile, and include header files:
ecat\_motor\_drive.h

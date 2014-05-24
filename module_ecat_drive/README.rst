EtherCAT Drive Module
=====================

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

   SYNAPTICON
This module provides a communication bridge between Ethercat and Motor
drive System; complex motor drive/control functionalities. The
implemtation receives/sends data to the Ethercat Master Application;
monitors the state of the motor drive, sensor drive and control servers;
runs State Machine; initiates, starts and executes/shutsdown a
particular operation requested from the Ethercat Master Application;
packs the sensor data information to be sent to Ethercat master
application.

To include this module add module\_ecat\_drive to USED\_MODULES in the
application/test makefile, and include header files:
ecat\_motor\_drive.h

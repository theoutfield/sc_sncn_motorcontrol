SOMANET Motor Control Hall Sensor Component
===========================================

:scope: General Use
:description: Hall sensor interface
:keywords: SOMANET
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300

Description
-----------

This component provides driver for the Hall sensor connected to the SOMANET IFM-Drive hardware module. The component provides Hall server thread which drives the Hall sensor, acquires position information and calculates velocity in a while loop; and provides client functions to configure Hall Server with number of pole pairs and max ticks; get position and velocity from the Hall server.

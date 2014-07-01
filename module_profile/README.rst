SOMANET Motor Control Profile Module
====================================

:scope: General Use
:description: SOMANET Motor Control Motion Profile tools
:keywords: SOMANET
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC300


Description
-----------

This module provides Motion profile generation and Profile control
functions for position/velocity/torque control.

The profile generator provides functions to define the acceleration,
deceleration, slopes in some cases, velocity; and generates a profile to
match these requirements. Once initialized with these parameters the
profile can be generated using respective profile\_generate function.
Also profile generation for quick stop actions are provided. Note: the
profiles generated are limited by parameters like max acceleration, max
profile velocity or max/min allowed target value for the profile.

The Profile control functions makes use of the profile generators to
create motion profiles and communicates the profile with the respective
control Server.

To include this module add module\_profile to USED\_MODULES in the
application/test makefile, and include header files: profile.h and
profile\_control.h

.. _lib_velocity_auto_tune:

===========================
BLDC Torque Control Library
===========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This library provides functions to automatically calculate the parameters of a velocity PID controller. There are two functions in this library namely init_velocity_auto_tuner() and velocity_controller_auto_tune(). These functions accept the following parameters as input:

- structure of type AutoTuneParam which contains velocity_auto_tuning parameters
- The reference velocity which will be used in auto_tuning procedure.
- Actual velocity of motor (in rpm) which is measured by position feedback service

Once autotuning is triggered, motion_control_service will control the velocity up a certain value (by default 50% of the rated speed). This step will take around 4 seconds, and after that, proper PID constants will be calculated to control the system in velocity mode. The calculated parameters will be automatically updated in motion_control_service().


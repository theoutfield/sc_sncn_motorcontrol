.. _bldc_motor_drive_demo:

=======================
SOMANET BLDC Motor Control Demo
=======================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_bldc_motorcontrol) is showing the use of the :ref:`Motor Control Module <module_motorcontrol>` over a BLDC motor. For that, it implements a simple app that commutate your BLDC motor using a Hall Effect feedback sensor. The app also will display over **XScope** the present currents on phases B and C.

* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_bldc_motorcontrol/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, motor phases, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_demo_bldc_motorcontrol**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

      .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Set the configuration for Motor Control and Hall Service.    (LINK TO HOW TO CONFIGURE) 
7. Run the application (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


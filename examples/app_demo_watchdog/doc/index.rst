.. _watchdog_driver_demo:

============================
SOMANET Watchdog Driver Demo
============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_watchdog) is showing the use of the :ref:`Watchdog Module <module_watchdog>`. For that, it implements a simple app that disables the Watchdog ticking while spinning the motor. After that it renables the Watchdog chip operation again.

* **Min. Nr. of cores**: 2
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_watchdog/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, motor phases, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_demo_watchdog**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?).
6. Set the configuration for Motor Control and Hall Service.    (LINK TO HOW TO CONFIGURE) 
7. Run the application (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


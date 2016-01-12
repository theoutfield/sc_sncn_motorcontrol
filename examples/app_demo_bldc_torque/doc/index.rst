.. _bldc_torque_control_demo:

============================
SOMANET BLDC Torque Control Demo
============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_bldc_torque) is showing the use of the :ref:`Control Loops Module <module_ctrl_loops>` and :ref:`Profile Module <module_profile>` for Torque control of a BLDC motor. For that, it implements a simple app that will make your motor reach a desired target torque, first on one direction and then on the opposite one. A Hall Effect Feedback sensor will be used for commutation and, additionally, an Encoder Interface can be used for accurate position feedback. The app also will display over **XScope** the currents on phases B and C together with the target torque.

* **Minimum Number of Cores**: 9
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_bldc_torque/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, Encoder Interface (if used), motor phases, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_demo_bldc_torque**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

    .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Set the configuration for Motor Control, Hall, Encoder (if used), and Torque Control Services. Also for your Torque Profiler.  (LINK TO HOW TO CONFIGURE) 
7. Run the application (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS).

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


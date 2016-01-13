.. _brushed_dc_position_control_demo:

=================================
SOMANET Brushed DC Position Control Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_brushed_dc_position) is showing the use of the :ref:`Control Loops Module <module_ctrl_loops>` and :ref:`Profile Module <module_profile>` for Position control of a Brushed DCmotor. For that, it implements a simple app that will make your motor reach a desired target position. An Encoder Interface will be used for position feedback. The app also will display over **XScope** the current position of the motor respect to the target position.

* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_brushed_dc_position/>`_

Quick How-to
============
1. `Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific hardware documentation. Connect your Encoder Interface, motor phases (A and B), power supply cable, and XTAG. Power up!
3. `Set up your XMOS development tools <getting_started_xmos_dev_tools>`_. 
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_demo_brushed_dc_position**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

    .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Set the configuration for Motor Control, Encoder, and Position Control Services. Also for your Position Profiler.  (LINK TO HOW TO CONFIGURE) 
7. Run the application (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS).

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP



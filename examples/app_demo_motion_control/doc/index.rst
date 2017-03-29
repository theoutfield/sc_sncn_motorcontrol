======================================================
Quick Start Guide for app_demo_motion_control
======================================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

Introduction
===============
In many industrial applications, it is required to control torque, velocity or position of an electrical motors. For example, torque control is used in electric vehicle applications, while velocity control is used in electric elevators, and position control is mostly used in robotic application.
In all of these applications, it is usually preferred to limit the acceleration rate of motor shaft to avoid mechanical stresses. This aim can be achieved to use a profiled reference (with limited acceleration rate) instead of a step command for reference values.

The application **app_demo_motion_control** demonstrates how to implement torque, velocity or position control with a profiled reference command. Using this application, the user will be able to:

- select between different motion control modes (torque/velocity/position modes or disable mode)
- send direct/step commands in each mode

* **Minimum Number of Cores**: 5
* **Minimum Number of Tiles**: 3

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your position sensor, motor phases, power supply cable, and XTAG. Power up!
   .. important:: For safety please use a current limited power supply and always monitor the current consumption.
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.

5. Open the **main.xc** within  the **app_demo_motion_control**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropriate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the configuration <motor_configuration_label>` for Motor Control, Hall, Encoder (if used), and Position Control Services. Also for your Position Profiler.
7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

The user will be asked for the proffered motion control mode (torque/velocity/position control). As shown in the next picture, it is also possible to switch between a step command and a direct command.
reference and real values of torque, velocity and position will be displayed on Xscope.

.. image:: images/step_and_direct_commands_in_profiled_modes_001.jpg
   :width: 90%

* **Minimum Number of Cores**: 7
* **Minimum Number of Tiles**: 3

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_motion_control/>`_


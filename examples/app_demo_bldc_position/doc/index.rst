.. _bldc_motion_control_demo:

========================
BLDC Motion Control Demo
========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_motion_control) is to show the use of the :ref:`Motion Control Module <module_motion_control>` and :ref:`Profile Module <module_profile>` for Position/Velocity/Torque control of a BLDC motor. For that, it implements a simple app that will prompt you to select desired control mode and a reference value to execute the motion. The app will also display over **XScope** the current position, velocity, and torque of the motor in respect to the reference value.

* **Minimum Number of Cores**: X
* **Minimum Number of Tiles**: X

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_demo_motion_control/>`_

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Hall sensor, Encoder Interface (if used), motor phases, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_bldc_position**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropriate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the configuration <motor_configuration_label>` for Motor Control, Hall, Encoder (if used), and Position Control Services. Also for your Position Profiler.
7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.


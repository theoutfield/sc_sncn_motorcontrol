.. _bldc_position_control_demo:

==================================
SOMANET BLDC Position Control Demo
==================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_bldc_position) is showing the use of the :ref:`Control Loops Module <module_ctrl_loops>` and :ref:`Profile Module <module_profile>` for Position control of a BLDC motor. For that, it implements a simple app that will make your motor reach a desired target position. A Hall Effect Feedback sensor will be used for commutation and, additionally, an Encoder Interface can be used for accurate position feedback and control. The app also will display over **XScope** the current position of the motor respect to the target position.

* **Minimum Number of Cores**: 8
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_bldc_position/>`_

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Hall sensor, Encoder Interface (if used), motor phases, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_bldc_position**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the configuration <motor_configuration_label>` for Motor Control, Hall, Encoder (if used), and Position Control Services. Also for your Position Profiler.
7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.


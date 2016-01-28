.. _brushed_dc_torque_control_demo:

======================================
SOMANET Brushed DC Torque Control Demo
======================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_brushed_dc_torque) is showing the use of the :ref:`Control Loops Module <module_ctrl_loops>` and :ref:`Profile Module <module_profile>` for Torque control of a Brushed DC motor. For that, it implements a simple app that will make your motor reach a desired target torque, first on one direction and then on the opposite one. An Encoder Interface will be used for position feedback. The app also will display over **XScope** the currents on phase B together with the target torque.

* **Minimum Number of Cores**: 8
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_brushed_dc_torque/>`_

Quick How-to
============
11. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Encoder Interface, motor phases (A and B), power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_brushed_dc_torque**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the configuration <motor_configuration_label>` for Motor Control, Encoder, and Torque Control Services. Also for your Torque Profiler. 
7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.


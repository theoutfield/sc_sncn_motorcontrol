.. _general_pwm_application_demo:

================================
General PWM Application Demo
================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_demo_general_pwm) is showing the use of pwm_service_general in :ref:`PWM Module <module_pwm>` in different SOMANET Drives. For this purpose, a simple function (named send_pwm_values) is defined. This function changes the pwm values in each execution loop, and sends them to general pwm service through its corresponding interface. 

* **Minimum Number of Cores**: 3
* **Minimum Number of Tiles**: 1

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect power supply cable and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_general_pwm**. Include the :ref:`board-support file according to your device <DRIVE_BOARD_REQUIRED>`, and Include the :ref:`Core-support file according to your Core module <CORE_BOARD_REQUIRED>`. Moreover, set the :ref:`appropriate core target in your Makefile.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

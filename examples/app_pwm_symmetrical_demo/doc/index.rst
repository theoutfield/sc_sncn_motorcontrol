.. _pwm_symmetrical_demo:

====================
PWM generation Demo
====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_pwm_symmetrical_demo) is showing the use of the :ref:`Symmetrical PWM Module <pwm_symmetrical_module>`. For that, it simple generates naive PWM over 6 different ports.

Enabling the Watchdog chip on your SOMANET DC board is required to output the PWM to the Phase terminals. And still, you will not see exactly the generated pulses, since these pulses modulate a Fet Driver, but not the phases directly. Therefore it will not be possible to read the PWM outputs physically in your hardware. 

Nevertheless, you can use the xTIMEcomposer simulator to check what is being produce by the application. 

* **Minimum Number of Cores**: 1
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_pwm_symmetrical_demo/>`_

Quick how-to
============

1. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
2. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
3. Run the application over the `simulator generating an output to a VCD trace file <https://www.xmos.com/published/xsimtut>`_. You will have to configure the simulator to trace the **ports** of **tile[3]**.
4. The app will run in an endless loop, terminate the **Run** after 2 minutes.
5. `Display the generated VCD file <https://www.xmos.com/published/xsimtut>`_ and check the following ports:
        * **XS1_PORT_1K** and **XS1_PORT_1L**: These are the high and low side respectively for phase A (coordinate 0 at PWM output vector). 
        * **XS1_PORT_1O** and **XS1_PORT_1P**: These are the high and low side respectively for phase B (coordinate 1 at PWM output vector). 
        * **XS1_PORT_1M** and **XS1_PORT_1N**: These are the high and low side respectively for phase C (coordinate 2 at PWM output vector). 

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

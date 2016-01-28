.. _qei_demo:
=================================
SOMANET Encoder Interface Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_test_qei) is showing the use of the :ref:`Encoder Interface Module <module_qei>`. For that, it implements a simple app that reads the output of an Encoder sensor and shows over **XScope** the read velocity and position.

* **Minimum Number of Cores**: 2
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_test_qei/>`_

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your Encoder sensor, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_test_qei**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropiate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. Again in your **main.xc**, set the configuration for your Encoder Service. 

.. code-block:: C

        on tile[IFM_TILE]:
        /* Quadrature Encoder sensor Service */
        {
                QEIConfig qei_config; 
                qei_config.signal_type = QEI_RS422_SIGNAL;              
                qei_config.index_type = QEI_WITH_INDEX;                 
                qei_config.ticks_resolution = 4000;                     
                qei_config.sensor_polarity = QEI_POLARITY_NORMAL;       

                qei_service(qei_ports, qei_config, i_qei);
        }

.. important:: Not all the SOMANET IFM DC boards support TTL Encoder output signals. Make sure your DC device support such configuration.

7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.


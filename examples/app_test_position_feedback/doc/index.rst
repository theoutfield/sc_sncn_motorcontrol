.. _biss_demo:

=================================
BiSS Interface Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_test_biss) is showing the use of the :ref:`BiSS Interface Module <module_biss>`. For that, it implements a simple app that reads the output of an BiSS sensor and shows over **XScope** the read velocity and position.

* **Min. Nr. of cores**: 2
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_test_biss/>`_

Quick How-to
============

1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your BiSS sensor, power supply cable, and XTAG. Power up!
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`.
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_test_biss**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropriate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. Again in your **main.xc**, set the configuration for your BiSS Service.

    .. code-block:: c

            on tile[IFM_TILE]:
            /* BiSS Service */
            {
                BISSConfig biss_config;
                biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                biss_config.status_length = BISS_STATUS_LENGTH;
                biss_config.crc_poly = BISS_CRC_POLY;
                biss_config.pole_pairs = 2;
                biss_config.polarity = BISS_POLARITY;
                biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                biss_config.timeout = BISS_TIMEOUT;
                biss_config.max_ticks = BISS_MAX_TICKS;
                biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                biss_service(biss_ports, biss_config, i_biss);
            }

7. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.

.. _biss_demo:
=================================
SOMANET BiSS Interface Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_test_biss) is showing the use of the :ref:`BiSS Interface Module <module_biss>`. For that, it implements a simple app that reads the output of an BiSS sensor and shows over **XScope** the read velocity and position.

* **Min. Nr. of cores**: 2
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_test_biss/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Encoder sensor, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_test_biss**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

   .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Again in your **main.xc**, set the configuration for your BiSS Service. 

.. code-block:: C

        on tile[IFM_TILE]:
        /* BiSS server */
        {
            BISSConfig biss_config;
            biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
            biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
            biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
            biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
            biss_config.status_length = BISS_STATUS_LENGTH;
            biss_config.crc_poly = BISS_CRC_POLY;
            biss_config.poles = 3;
            biss_config.polarity = BISS_POLARITY;
            biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
            biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
            biss_config.timeout = BISS_TIMEOUT;
            biss_config.max_ticks = BISS_MAX_TICKS;
            biss_config.velocity_loop = BISS_VELOCITY_LOOP;
            biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

            biss_service(biss_ports, biss_config, i_biss);
        }


6. Run the application enabling XScope (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


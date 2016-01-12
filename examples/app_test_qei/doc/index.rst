.. _qei_demo:
=================================
SOMANET Encoder Interface Demo
=================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_test_qei) is showing the use of the :ref:`Encoder Interface Module <module_qei>`. For that, it implements a simple app that reads the output of an Encoder sensor and shows over **XScope** the read velocity and position.

* **Min. Nr. of cores**: 2
* **Min. Nr. of tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_test_qei/>`_

Quick How-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Encoder sensor, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_test_qei**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

   .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

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

6. Run the application enabling XScope (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


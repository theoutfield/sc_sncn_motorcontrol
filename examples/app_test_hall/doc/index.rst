.. _hall_demo:

==========================================
SOMANET Hall Effect Feecback Sensor Demo
==========================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app (app_test_hall) is showing the use of the :ref:`Hall Sensor Module <module_hall>`. For that, it implements a simple app that reads the output of a Hall Effect sensor and shows over **XScope** the velocity and absolute position. Over console, it will output the electrical position of your shaft.

* **Minimum Number of Cores**: 2
* **Minimum Number of Tiles**: 2

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_test_hall/>`_

Quick How-to
============

1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, power supply cable, and XTAG. Power up!
3. Set up your development environment by installing xTIMEcomposer. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Control Library and its dependancies.
5. Open the **main.xc** within  the **app_test_hall**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?). Also set the appropiate target in your Makefile. (LINK HOW TO SET YOUR RIGHT TARGET IN YOUR MAKEFILE)

   .. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the **Hardware compatibility** section of the library. (LINK TO IT).

6. Again in your **main.xc**, set the configuration for your Hall Service. 

    .. code-block:: C

            on tile[IFM_TILE]:
            /* Hall Service */
            {
                HallConfig hall_config;
                hall_config.pole_pairs = 1;

                hall_service(hall_ports, hall_config, i_hall);
            }

    .. note:: Do not worry if you don't know how many pole pairs your motor has. You can always run the application and see on the **console outputs** how many **electrical revolutions** are contained in a mechanical motor revolution. **This number is the number of pole pairs in your motor**.

7. Run the application enabling XScope (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


=========================
Feedback Hall Sensor Demo
=========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

The purpose of this app is showing the use of the :ref:`Hall Sensor Module <module_hall>`. For that, it implements a simple app that reads the output of a Hall Effect sensor and shows over **XScope** the velocity and absolute position. Over console, it will output the electrical position of your shaft.

Min. Nr. of cores: **2**.
Min. Nr. of tiles: **2**.

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/develop/examples/app_test_hall/>`_

Quick how-to
============
1. **Assemble your SOMANET device** (LINK TO TUTORIAL PAGE).
2. **Wire up your device** (LINK TO INTERFACING YOUR SOMANET). Connect your Hall sensor, power supply cable, and XTAG. Power up!
3. Install xTIMEcomposer to set your development environment. (LINK TO TUTORIAL OR TO XMOS TUTORIAL)
4. Download and **import** (LINK TO TUTORIAL: IMPORTING SOMANET LIBRARIES) in your workspace the SOMANET Motor Contrl Library and its dependancies.
5. Open the **main.xc** within  the **app_test_hall**. Include the **board-support file according to your device** (LINK TO BOARD SUPPORT MODULE?) and set the configuration for your Hall Service. 

.. code-block:: C

        on tile[IFM_TILE]:
        /* Hall Service */
        {
            HallConfig hall_config;
            hall_config.pole_pairs = 3;

            hall_service(hall_ports, hall_config, i_hall);
        }

.. note:: If you do not know how many pole pairs your motor has, do not worry. You can always run the application and see on the console outputs how many electrical revolutions are contained in a mechanical motor revolution. This number is the number of pole pairs in your motor.

6. Run the application enabling XScope (LINK TO TUTORIAL HOW TO RUN SOMANET APPLICATIONS)

HERE WE SHOULD PROVIDE MAYBE LINKS TO THE FORUM OR SOME SUPPORT... IF SOMETHING GOES WRONG WHILE RUNNING THE APP


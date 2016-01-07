===========================
SOMANET Hall Sensor Module
===========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from your 
Feedback Hall Sensor. All the read data and related calculations can be accessed
by a client through an interface.

.. image:: images/core-diagram-hall-interface.png
   :width: 50%

How to use
==========

Getting position and velocity information from your Hall sensor
---------------------------------------------------------------

Step 1: Include the required modules & headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Make sure you Makefile contains at least these module

.. code-block:: C

    USED_MODULES = module_blocks module_hall module_motor module_motorcontrol_common module_board-support

Make sure you include these files in your main.xc file

.. code-block:: C

    #include <xs1.h>
    #include <platform.h>
    #include <ioports.h>
    #include <hall_client.h>
    #include <hall_service.h>


Step 2: Define required channel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
A channel is required to transport data from the hall_server task to your custom client's task

.. code-block:: C

    int main(void)
    {
        chan c_hall
        ...
    }


Step 4: Run required tasks/servers: PWM, Commutation, Watchdog and Hall interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. important:: Please note that all these tasks must be executed on a tile with access to I/O of a Synapticon SOMANET IFM Drive DC board. 

.. code-block:: C

    int main(void)
    {
    ...

        par
        {
        ...

            on tile[IFM_TILE]:
            {
                par
                {
                    /* Hall Server */
                    {
                        hall_par hall_params;
                        run_hall(c_hall, NULL, NULL, NULL, NULL, NULL, p_ifm_hall, hall_params); // channel priority 1,2..6
                    }
                }
            }
            ...

        }

        return 0;
    }


Using hall_client to get velocity/position information
------------------------------------------------------
Getting velocity and position information from the hall server is easy:

.. code-block:: C

    int main(void)
    {
    ...

        par
        {
            ...

            on tile[0]: // Can be any tile
            {
                /* Get position from Hall Sensor */
                {position, direction} = get_hall_position_absolute(c_hall);

                /* Get velocity from Hall Sensor */
                velocity = get_hall_velocity(c_hall);
            }
        }
        return 0;
    }


API
===

Types
-----

.. doxygenstruct:: HallConfig
.. doxygenstruct:: HallPorts

Service
-------

.. doxygenfunction:: hall_service

Interface
---------

.. doxygeninterface:: HallInterface

.. _qei_programming_label:

Programming Guide
=================

Getting position and velocity information from your Quadrature Encoder
----------------------------------------------------------------------

Step 1: Include the required modules & headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Make sure you Makefile contains at least these module

::

    USED_MODULES = module_blocks module_qei module_motor module_motorcontrol_common module_nodeconfig

Make sure you include these files in your main.xc file

::

    #include <xs1.h>
    #include <platform.h>
    #include <ioports.h>
    #include <qei_client.h>
    #include <qei_server.h>


Step 2: Define required channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
A channel is required to transport data from the qei_server task to your custom client's task

::

	int main(void)
	{
		chan c_qei
		...
	}


Step 4: Run required tasks/servers: PWM, Commutation, Watchdog and Quadrature Encoder Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. important:: Please note that all these tasks must be executed on a tile with access to I/O of a Synapticon SOMANET IFM Drive DC board. 

::

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
                    /* QEI Server */
                    {
                        qei_par qei_params;
                        run_qei(c_qei, NULL, NULL, NULL, NULL, NULL, p_ifm_encoder, qei_params);
                    }
                }
            }
            ...

        }

        return 0;
    }


Getting velocity/position information from sensor server
--------------------------------------------------------
While the QEI server is running and constantly monitoring, the velocity and position can be aquired by a simple API call:

::

    int main(void)
    {
    ...

        par
        {
            on tile[0]: // Can be any tile
            {
                /* Get encoder absolute position */
                {position, direction} = get_qei_position_absolute(c_qei);

                /* Get current velocity */
                velocity = get_qei_velocity(c_qei,);
            }
        }
    ...

    }

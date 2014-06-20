.. _commutation_programming_label:

Programming Guide
=================

Running the hall server
------------------------------

Step 1: Include the required headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    #include <xs1.h>
    #include <platform.h>
    #include <ioports.h>
    #include <hall_server.h>
    #include <pwm_service_inv.h>
    #include <comm_loop_server.h>
    #include <refclk.h>
    #include <drive_modes.h>
    #include <statemachine.h>
    #include <internal_config.h>
    #include <bldc_motor_config.h>


Step 2: Define required channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

	int main(void)
	{
		chan c_hall

		...
	}


Step 4: Run required tasks/servers: PWM, Commutation, Watchdog and Hall interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
                    /* Hall Server */
                    {
                        hall_par hall_params;
                        run_hall(c_hall, NULL, NULL, NULL, NULL, NULL, p_ifm_hall, hall_params); // channel priority 1,2..6
                    }
                }
            }
    
        }


        ...
    

        return 0;
    }


Getting velocity/position information from sensor server
--------------------------------------------------------
Getting velocity and position information from the hall server is easy:
::

    int main(void)
    {

    ...

        par
        {
            on tile[0]: // Can be any tile
            {
                /* Get position from Hall Sensor */
                {position, direction} = get_hall_position_absolute(c_hall);

                /* Get velocity from Hall Sensor */
                velocity = get_hall_velocity(c_hall, hall_params);
            }
        }

    ...

    }

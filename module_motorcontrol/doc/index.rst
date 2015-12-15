==================================
SOMANET BLDC Motorcontrol Module
==================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

Like shown in the core diagram below, 4 timing critical task are executed on the SOMANET IFM Tile:

* **Commutation Server** - Executes sinusoidal commutation software
* **PWM Server** - Generates PWM signals
* **Hall Server** - Delivers rotor angle information required for sinusoidal commutation
* **Watchdog Server** - Provides WDT (Watchdog timer) functionality

As shown, custom controllers should run in a different tile to avoid influencing the performance of motor control. Please refer to the :ref:`commutation_programming_label`


.. figure:: images/core-diagram-commutation.png
   :width: 60%

   Custom control application core diagram

.. _commutation_programming_label:

How to use
===========

Running the commutation server
------------------------------

Step 1: Include the required headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: C

    #include <xs1.h>
    #include <platform.h>
    #include <ioports.h>
    #include <hall_server.h>
    #include <pwm_service_inv.h>
    #include <commutation_server.h>
    #include <refclk.h>
    #include <drive_modes.h>
    #include <statemachine.h>
    #include <internal_config.h>
    #include <bldc_motor_config.h>

Step 2: Define the clocks
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: C

    on tile[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;
    on tile[IFM_TILE]: clock clk_pwm = XS1_CLKBLK_REF;

Step 3: Define motor control channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: C

	int main(void)
	{
		chan c_qei_p1;                                                          // qei channel
		chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;  // hall channels
		chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;    // commutation channels
		chan c_pwm_ctrl, c_adctrig;                                             // pwm channels
		chan c_watchdog;

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
                    /* PWM Loop */
                    do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
                            p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
    
                    /* Motor Commutation loop */
                    {
                        hall_par hall_params;
                        qei_par qei_params;
                        commutation_par commutation_params;
                        commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog,  \
                                c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl,\
                                p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2,\
                                hall_params, qei_params, commutation_params);
                    }
    
                    /* Watchdog Server */
                    run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);
    
                    /* Hall Server */
                    {
                        hall_par hall_params;
                        run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
                    }
                }
            }
    
        }


        ...
    

        return 0;
    }


Applying output voltage to the motor
-------------------------------------
A motor voltage can be applied by a simple function call:

.. code-block:: C

    int main(void)
    {

    ...

        par
        {
            on tile[0]: // Can be any tile
            {
                //Set commutation value to 100
                set_commutation_sinusoidal(c_commutation_p1, 100);
            }
        }

    ...

    }

.. tip:: Start by applying lower values like e.g. 20 before causing any demage to your motor or driver.

.. note:: The commutation value's range is -13739 to 13739


API
====

Types
-----

.. doxygenenum:: MotorType
.. doxygenstruct:: FetDriverPorts
.. doxygenstruct:: MotorcontrolConfig

Service
--------

.. doxygenfunction:: motorcontrol_service

Interface
---------

.. doxygeninterface:: MotorcontrolInterface


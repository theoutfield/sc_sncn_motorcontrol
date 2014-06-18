Programming Guide
=================

Running the commutation server
---------------------------------

Step 1: Define motor control channels
.............................

::

	int main(void)
	{
		chan c_qei_p1;                                                          // qei channel
		chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;  // hall channels
		chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;    // commutation channels
		chan c_pwm_ctrl, c_adctrig;                                             // pwm channels
		chan c_watchdog;

		...
	}


Step 2: Run PWM, commutation, watchdog and hall tasks on a tile with access to SOMANET IFM I/O
...............................................................................................

::

	int main(void)
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
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
					commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, 	\
							c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl,\
							p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2,\
							hall_params, qei_params, commutation_params);
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
				}
			}
		}

	...

	}


Applying output voltage to the motor
------------------------------------
A motor voltage can be applied by a simple function call:
::

    set_commutation_sinusoidal(c_commutation, target_output_voltage);

The target value's range is -13739 to 13739
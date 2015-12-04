/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <hall_server.h>
#include <qei_server.h>
#include <biss_server.h>
#include <pwm_service_inv.h>
#include <brushed_dc_server.h>
#include <brushed_dc_client.h>
#include <refclk.h>
#include <velocity_ctrl_client.h>
#include <velocity_ctrl_server.h>
#include <xscope.h>
#include <internal_config.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <profile_control.h>
#include <qei_client.h>
#include <profile.h>
#include <watchdog.h>
#include <bldc_motor_config.h>

on tile[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;
on tile[IFM_TILE]: clock clk_pwm = XS1_CLKBLK_REF;
on tile[IFM_TILE]: clock clk_biss = XS1_CLKBLK_2 ;
port out p_ifm_biss_clk = GPIO_D0;

interface virtual_master
{
    void set_velocity(int velocity_sp);
    void set_acceleration(int acc);
    void set_deceleration(int dec);
    int get_velocity(void);
};

/* Velocity Setpoints commanding task */
[[combinable]]
void profile_velocity_master(client interface virtual_master i_vm){
    int velocity_sp = 2000;
    timer tmr;
    long unsigned int time = 0;
    i_vm.set_acceleration(600);
    i_vm.set_deceleration(600);

    while(1){
        select{
            case tmr when timerafter(time + SEC_STD*50) :> time:
                i_vm.set_velocity(velocity_sp);
                velocity_sp *= -1;
            break;
        }
    }
}

/* Profile Velocity execution task */
[[combinable]]
void profile_velocity_slave(chanend c_velocity_ctrl, server interface virtual_master i_vm)
{
	int target_velocity = 0;	 		// rpm
	int acceleration 	= 0;			// rpm/s
	int deceleration 	= 0;			// rpm/s
    timer tmr;
    unsigned int time = 0;
    int steps = 0, steps_increment = 1, velocity_ramp = 0, actual_velocity = 0;
    profile_velocity_param profile_velocity_params;

    int init_state = init_velocity_control(c_velocity_ctrl);
    printf("initialized: %d\n", init_state);

	while(1){
	    select {
	        //Execute Profile Ramp
            case tmr when timerafter(time + MSEC_STD) :> time:
                    if(steps_increment < steps) {
                        velocity_ramp = __velocity_profile_generate_in_steps(steps_increment, profile_velocity_params);
                        set_velocity(velocity_ramp, c_velocity_ctrl);
                        actual_velocity = get_velocity(c_velocity_ctrl);
                        xscope_int(PROFILE, velocity_ramp);
                        xscope_int(VELOCITY, actual_velocity);
                        steps_increment++;
                    }
                    else if (target_velocity == 0) {
                        set_velocity(target_velocity, c_velocity_ctrl);
                    }
                break;
            case i_vm.set_velocity(int velocity_sp):
                    actual_velocity = get_velocity(c_velocity_ctrl);
                    target_velocity = velocity_sp;
                    steps = __initialize_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, profile_velocity_params);
                    steps_increment = 1;
                break;
            case i_vm.set_acceleration(int acc):
                    acceleration = acc;
                break;
            case i_vm.set_deceleration(int dec):
                    deceleration = dec;
                break;
            case i_vm.get_velocity(void) -> int velocity:
                    velocity = get_velocity(c_velocity_ctrl);
                    break;
	    }
	}
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_hall_p6, c_qei_p6;		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;						// hall channels
	chan c_commutation;						// motor drive channels
	chan c_pwm_ctrl, c_adctrig;														// pwm channels
	chan c_velocity_ctrl;															// velocity control channel
	chan c_watchdog; 																// watchdog channel
	interface virtual_master i_vm;
    interface i_biss i_biss[1];                                                     // biss interfaces

	par
	{
		/* Test Profile Velocity function */
		on tile[APP_TILE]:
		{
		    [[combine]]
		    par{
		        profile_velocity_master(i_vm);
			    profile_velocity_slave(c_velocity_ctrl, i_vm);			// test PVM on node
			}
		}

		on tile[APP_TILE]:
		{
            /* Velocity Control Loop */
            {
                ControlConfig velocity_ctrl_params;
                /* Initialize PID parameters for Velocity Control (defined in config/motor/bldc_motor_config.h) */
                init_velocity_control_config(velocity_ctrl_params);

                /* Control Loop */
                velocity_control_service(velocity_ctrl_params, i_hall[1], i_qei[1],
                                            i_velocity_control, i_commutation[0]);
            }
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
            par
            {
                /* PWM Loop */
                pwm_service(c_pwm_ctrl, pwm_ports);

                /* Watchdog Server */
                watchdog_service(i_watchdog, wd_ports);

                /* QEI Service */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(i_qei, qei_ports, qei_config);
                }

                {
                    MotorcontrolConfig commutation_config;
                    init_commutation_config(commutation_config);

                    motorcontrol_service(null, i_qei[0], i_watchdog, i_motorcontrol,
                                            c_pwm_ctrl, fet_driver_ports, commutation_config);
                }

            }
		}

	}

	return 0;
}

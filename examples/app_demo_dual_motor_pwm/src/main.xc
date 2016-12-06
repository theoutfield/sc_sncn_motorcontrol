/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

//#include <pwm_service.h>
#include <pwm_server.h>
#include <user_config.h>
#include <watchdog_service.h>

#include <xscope.h>
#include <timer.h>

PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

void send_pwm_values(client interface update_pwm_general i_update_pwm)
{
    timer t;
    unsigned start_time=0, end_time=0, total_period=0;
    unsigned int t_claculation=0;
    int pwm_value=2000;

    while(1)
    {
        pwm_value=2000;
        t :> start_time;
        t_claculation = i_update_pwm.update_server_control_data(
                        /*int pwm_a*/pwm_value, /*int pwm_b*/pwm_value, /*int pwm_c*/pwm_value,
                        /*int pwm_u*/pwm_value, /*int pwm_v*/pwm_value, /*int pwm_w*/pwm_value,
                        /*int received_pwm_on*/0, /*int received_brake_active*/0, /*int recieved_safe_torque_off_mode*/0);
        t :> end_time;
        delay_milliseconds(500);



        pwm_value=3000;
        t :> start_time;
        t_claculation = i_update_pwm.update_server_control_data(
                        /*int pwm_a*/pwm_value, /*int pwm_b*/pwm_value, /*int pwm_c*/pwm_value,
                        /*int pwm_u*/pwm_value, /*int pwm_v*/pwm_value, /*int pwm_w*/pwm_value,
                        /*int received_pwm_on*/0, /*int received_brake_active*/0, /*int recieved_safe_torque_off_mode*/0);
        t :> end_time;
        delay_milliseconds(500);
    }
}

int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface update_pwm_general i_update_pwm;

    par
    {
         on tile[IFM_TILE]:
        {
            par
            {
                {
                    send_pwm_values(i_update_pwm);
                }
                /* PWM Service */
                {
                    pwm_config_general(pwm_ports, IFM_TILE_USEC);

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    pwm_service_general(
                            pwm_ports, i_update_pwm,
                            DUTY_START_BRAKE, DUTY_MAINTAIN_BRAKE, PERIOD_START_BRAKE,
                            IFM_TILE_USEC, COMMUTATION_FRQ);

                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                }
            }
        }
    }

    return 0;
}

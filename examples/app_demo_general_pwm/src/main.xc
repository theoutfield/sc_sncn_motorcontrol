/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file main.xc
 * @brief Demo application illustrates usage of module_pwm
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <pwm_server.h>
#include <user_config.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <watchdog_service.h>

#include <xscope.h>
#include <timer.h>

void ocupy_core(int foo)//just a while(1) loop to ocupy the core, and increase computational load of cpu
{
    int x=0;
    int y=0;

    while(1)
    {
        x++;
        y++;
        if(x>100) x=foo;
        if(y>100) y=2*foo;
    }
}

//Sends pwm values for 6 nullable inverter outputs to general pwm service. The updating rate is 10 kHz
void send_pwm_values(client interface UpdatePWMGeneral i_update_pwm)
{
    timer t;
    unsigned int time=0x00000000;
    unsigned int updating_period = GPWM_UPDATING_PERIOD;

    unsigned short  pwm_value_a = 0x0000, pwm_value_b = 0x0000, pwm_value_c = 0x0000,
                    pwm_value_u = 0x0000, pwm_value_v = 0x0000, pwm_value_w = 0x0000;

    int counter=0;

    unsigned short pwm_value =0;
    unsigned short safe_torque_off=0x0000;

    time    =0x00000000;
    t :> time;
    while(1)
    {
        select
        {
        case t when timerafter(time) :> void:

            counter++;
            if(counter==50)
            {
                pwm_value ++;
                if(pwm_value>GPWM_MAX_VALUE_15_kHz)
                {
                    pwm_value=GPWM_MIN_VALUE;
                }
                counter=0;
            }

            pwm_value_a = pwm_value;
            pwm_value_b = pwm_value;
            pwm_value_c = pwm_value;
            pwm_value_u = pwm_value;
            pwm_value_v = pwm_value;
            pwm_value_w = pwm_value;

            xscope_int(PWM_VALUE_A, pwm_value_a);
            xscope_int(PWM_VALUE_B, pwm_value_b);
            xscope_int(PWM_VALUE_C, pwm_value_c);
            xscope_int(PWM_VALUE_U, pwm_value_u);
            xscope_int(PWM_VALUE_V, pwm_value_v);
            xscope_int(PWM_VALUE_W, pwm_value_w);

            pwm_value_a &= 0x0000FFFF;
            pwm_value_b &= 0x0000FFFF;
            pwm_value_c &= 0x0000FFFF;
            pwm_value_u &= 0x0000FFFF;
            pwm_value_v &= 0x0000FFFF;
            pwm_value_w &= 0x0000FFFF;

            i_update_pwm.update_server_control_data(
                    /*unsigned short pwm_a*/pwm_value_a, /*unsigned short pwm_b*/pwm_value_b, /*unsigned short pwm_c*/pwm_value_c,
                    /*unsigned short pwm_u*/pwm_value_u, /*unsigned short pwm_v*/pwm_value_v, /*unsigned short pwm_w*/pwm_value_w);

            time     += updating_period;
            break;
        }
    }
}

PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

int main(void) {

    interface UpdatePWMGeneral i_update_pwm;
    interface WatchdogInterface i_watchdog[2];

    par
    {
        on tile[0]://APP TILE
        {
            par
            {
                {
                    ocupy_core(10);
                }
                {
                    ocupy_core(11);
                }
                {
                    ocupy_core(12);
                }
                {
                    ocupy_core(13);
                }
                {
                    ocupy_core(14);
                }
                {
                    ocupy_core(15);
                }
                {
                    ocupy_core(16);
                }
                {
                    ocupy_core(17);
                }
            }
        }

        on tile[1]://IFM TILE
        {
            par
            {

                {
                    delay_milliseconds(20);
                    send_pwm_values(i_update_pwm);
                }

                /* PWM Service */
                {
                    pwm_config_general(pwm_ports);

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    pwm_service_general(pwm_ports, i_update_pwm, GPWM_FRQ_15, DEADTIME_NS);
                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                }

                {
                    ocupy_core(22);
                }
                {
                    ocupy_core(23);
                }
                {
                    ocupy_core(24);
                }
                {
                    ocupy_core(25);
                }

            }
        }
    }

    return 0;
}

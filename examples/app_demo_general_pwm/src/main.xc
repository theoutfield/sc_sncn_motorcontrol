/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C21-DX_G2.bsp>
#include <IFM_DC1K-rev-c4.bsp>

/**
 * @file main.xc
 * @brief Demo application illustrates usage of module_pwm
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <pwm_server.h>

#include <xscope.h>
#include <timer.h>

PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;

void ocupy_core(int foo)//just a while(1) loop to ocupy the core, and increase computational load of cpu
{
    int x=0;
    int y=0;

    x=10 * foo + 1;
    y=13 * foo + 5;

    while(1)
    {
        for(int i=100;i<=1000000;i++)
        {
            y = y*x + i;
            x = x + foo;
            x = x + 10;
            x = x * y;

            if (y>10000) y = 13 * foo + 5;
            if (x>10000) x = 10 * foo + 1;
        }

        y++;
        x++;
        y+=foo;
        x+=foo;
    }
}

//Sends pwm values for 6 nullable inverter outputs to general pwm service. The updating rate is 10 kHz
void send_pwm_values(client interface UpdatePWMGeneral i_update_pwm)
{
    timer t;
    unsigned int time=0x00000000;
    unsigned int updating_period = GPWM_MAX_VALUE;

    unsigned short  pwm_value_a = 0x0000, pwm_value_b = 0x0000, pwm_value_c = 0x0000,
                    pwm_value_u = 0x0000, pwm_value_v = 0x0000, pwm_value_w = 0x0000;

    short pwm_delta =0x0000;
    unsigned short pwm_value=0;

    unsigned short pwm_limit_low  = 0x0000;
    unsigned short pwm_limit_high = 0x0000;

    pwm_limit_high= GPWM_LIMIT_HIGH;
    pwm_limit_low = GPWM_LIMIT_LOW;

    pwm_delta = 1;
    pwm_value = pwm_limit_low;

    time    =0x00000000;
    t :> time;
    while(1)
    {
        select
        {
        case t when timerafter(time) :> void:

             pwm_value ++;
             if(pwm_value>pwm_limit_high)
             {
                 pwm_value=pwm_limit_low;
             }

             pwm_value_a = pwm_value;
             pwm_value_b = pwm_value;
             pwm_value_c = pwm_value;
             pwm_value_u = pwm_value;
             pwm_value_v = pwm_value;
             pwm_value_w = pwm_value;


             pwm_value_a &= 0x0000FFFF;
             pwm_value_b &= 0x0000FFFF;
             pwm_value_c &= 0x0000FFFF;
             pwm_value_u &= 0x0000FFFF;
             pwm_value_v &= 0x0000FFFF;
             pwm_value_w &= 0x0000FFFF;
             i_update_pwm.update_server_control_data(
                     /*unsigned short pwm_a*/pwm_value_a, /*unsigned short pwm_b*/pwm_value_b, /*unsigned short pwm_c*/pwm_value_c,
                     /*unsigned short pwm_u*/pwm_value_u, /*unsigned short pwm_v*/pwm_value_v, /*unsigned short pwm_w*/pwm_value_w,
                     /*received_pwm_on (not activated)*/0, /*recieved_safe_torque_off_mode  (not activated)*/0);

             time     += updating_period;
            break;
        }
    }
}

int main(void) {

    interface UpdatePWMGeneral i_update_pwm;

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
                /* PWM Service */
                {
                    pwm_config_general(pwm_ports);

                    delay_milliseconds(10);
                    pwm_service_general(pwm_ports, i_update_pwm);
                }

                {
                    delay_milliseconds(20);
                    send_pwm_values(i_update_pwm);
                }

                {
                    ocupy_core(20);
                }
                {
                    ocupy_core(21);
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

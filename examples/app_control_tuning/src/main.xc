/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C21-DX_G2.bsp>
#include <IFM_DC1K-rev-d1.bsp>

/**
 * @file main.xc
 * @brief Demo application illustrates usage of module_pwm
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning_console.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <position_feedback_service.h>

#include <xscope.h>
#include <timer.h>

PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;


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
    unsigned int time_end=0x00000000, time_start=0x00000000, time_free=0x00000000;
    unsigned int updating_period = GPWM_MAX_VALUE;

    unsigned short  pwm_value_a = 0x0000, pwm_value_b = 0x0000, pwm_value_c = 0x0000,
                    pwm_value_u = 0x0000, pwm_value_v = 0x0000, pwm_value_w = 0x0000;

    int counter=0;
    int pulse_counter=0;
    int pulse_index=1;

    short pwm_delta =0x0000;
    unsigned short pwm_value =0;
    unsigned short gpwm_value=0;
    unsigned short delta_duty=1110;//1110;//1110 2220 3330 4440 5550 6660

    unsigned short pwm_limit_low  = 0x0000 & 0x0000FFFF;
    unsigned short pwm_limit_high = 0x0000 & 0x0000FFFF;

    int pwm_on         =0x00000001;
    int safe_torque_off=0x00000000;

    pwm_limit_high= GPWM_MAX_VALUE;
    pwm_limit_low = GPWM_MIN_VALUE;

    pwm_delta = 1;
    pwm_value = pwm_limit_low;

    time    =0x00000000;
    t :> time;
    while(1)
    {
        select
        {
        case t when timerafter(time) :> void:
            t :> time_start;
            time_free = time_start - time_end;

            /*
            counter++;
            if(counter==500)
            {
                gpwm_value ++;
                if(pwm_value>GPWM_MAX_VALUE)
                {
                    gpwm_value=GPWM_MIN_VALUE;
                }
                counter=0;
            }
            */

            /*
            if(pulse_index==1)
            {
                pulse_index=2;
                pwm_value  =2*gpwm_value/3;
            }
            else if(pulse_index==2)
            {
                pulse_index=3;
                pwm_value  =3*gpwm_value/3;
            }
            else if(pulse_index==3)
            {
                pulse_index=1;
                pwm_value  =1*gpwm_value/3;
            }
            */

            /*
            pwm_value = gpwm_value;
            pwm_value_a = pwm_value;
            pwm_value_b = pwm_value;
            pwm_value_c = pwm_value;
            pwm_value_u = pwm_value;
            pwm_value_v = pwm_value;
            pwm_value_w = pwm_value;
            */

            //// ------------------------------------------
            //pulse_counter=pulse_index;
            //p <: 0;
            //for(int k=1;k<pulse_counter ;k++)
            //{
            //    for(int j=0;j<=15;j++) p <: 1;
            //    for(int j=0;j<=15;j++) p <: 0;
            //}
            //for(int j=0;j<=15;j++) p <: 1;
            //p <: 0;
            //// ------------------------------------------


            pwm_value_a += delta_duty;
            if(pwm_value_a>pwm_limit_high){
                pwm_value_a = pwm_limit_low;

                pwm_value_b += delta_duty;
                if(pwm_value_b>pwm_limit_high){
                    pwm_value_b = pwm_limit_low;

                    pwm_value_c += delta_duty;
                    if(pwm_value_c>pwm_limit_high){
                        pwm_value_c = pwm_limit_low;

                        pwm_value_u += delta_duty;
                        if(pwm_value_u>pwm_limit_high){
                            pwm_value_u = pwm_limit_low;

                            pwm_value_v += delta_duty;
                            if(pwm_value_v>pwm_limit_high){
                                pwm_value_v = pwm_limit_low;

                                pwm_value_w += delta_duty;
                                if(pwm_value_w>pwm_limit_high){
                                    pwm_value_w = pwm_limit_low;
                                }
                            }
                        }
                    }
                }
            }

            xscope_int(PWM_VALUE_A, pwm_value_a-GPWM_MAX_VALUE);
            xscope_int(PWM_VALUE_B, pwm_value_b-GPWM_MAX_VALUE);
            xscope_int(PWM_VALUE_C, pwm_value_c-GPWM_MAX_VALUE);
            xscope_int(PWM_VALUE_U, pwm_value_u-GPWM_MAX_VALUE);
            xscope_int(PWM_VALUE_V, pwm_value_v-GPWM_MAX_VALUE);
            xscope_int(PWM_VALUE_W, pwm_value_w-GPWM_MAX_VALUE);
            xscope_int(TIME_FREE, time_free);

            pwm_value_a &= 0x0000FFFF;
            pwm_value_b &= 0x0000FFFF;
            pwm_value_c &= 0x0000FFFF;
            pwm_value_u &= 0x0000FFFF;
            pwm_value_v &= 0x0000FFFF;
            pwm_value_w &= 0x0000FFFF;

            i_update_pwm.update_server_control_data(
                    /*unsigned short pwm_a*/pwm_value_a, /*unsigned short pwm_b*/pwm_value_b, /*unsigned short pwm_c*/pwm_value_c,
                    /*unsigned short pwm_u*/pwm_value_u, /*unsigned short pwm_v*/pwm_value_v, /*unsigned short pwm_w*/pwm_value_w,
                    /*pwm_on              */pwm_on     , /*safe_torque_off_mode*/safe_torque_off);

            time     += updating_period;

            t :> time_end;
            break;
        }
    }
}

int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
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
                    pwm_service_general(pwm_ports, i_update_pwm, 15);
                }

                {
                    delay_milliseconds(20);
                    send_pwm_values(i_update_pwm);
                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
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


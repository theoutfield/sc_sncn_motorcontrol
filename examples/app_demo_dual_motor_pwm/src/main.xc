/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

//#include <pwm_service.h>
#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <position_feedback_service.h>


PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

void send_pwm_values(
        client interface update_pwm_general i_update_pwm,
        interface WatchdogInterface client i_watchdog,
        int ref_clk_frq,
        int pwm_clk_frq,
        int commutation_frq)
{
    timer t;
    unsigned time=0, ts=0;
    int safe_torque_off_mode=0;
    int pwm_on=0;

    int pwm_values[6];
    int pwm_max=0, pwm_min=0, pwm_dif=0, pwm_av=0;

    int pwm_counter=0;
    int pwm_value=2000;

    unsigned int sync_inc=0;


    //proper task startup
    t :> ts;
    t when timerafter (ts + (5000*20*ref_clk_frq)) :> void;

    while(i_watchdog.status()!=ACTIVE);
    while(i_update_pwm.status()!=ACTIVE);

    if(ref_clk_frq==100)
    {
        if(pwm_clk_frq==100)
        {
            if(commutation_frq==12)
            {
                sync_inc= 8192;
                pwm_max = 7000;
                pwm_min = 600;
            }
            else if (commutation_frq==24)
            {
                sync_inc= 8192;
                pwm_max = 3200;
                pwm_min =  600;
            }
            else
            {
                printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
                while(1);
            }
        }
        else
        {
            printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
            while(1);
        }
    }
    else if(ref_clk_frq==250)
    {
        if(pwm_clk_frq==250)
        {
            if(commutation_frq==15)
            {
                sync_inc= 16384;
                pwm_max = 13000;
                pwm_min = 1500;
            }
            else
            {
                printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
                while(1);
            }
        }
        else
        {
            printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
            while(1);
        }
    }
    else
    {
        printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
        while(1);
    }

    pwm_dif =  pwm_max -  pwm_min;
    pwm_av  =( pwm_max +  pwm_min)/2;

    pwm_values[0] =  pwm_av;
    pwm_values[1] =  pwm_av;
    pwm_values[2] =  pwm_av;

    t :> time;
    while (1)
    {
        select
        {
        case t when timerafter(time) :> void:

            pwm_counter++;
            if(pwm_counter>10)
            {
                pwm_counter=0;

                pwm_value++;
                if(pwm_value>pwm_max) pwm_value= pwm_min;

                if(pwm_value<pwm_min) pwm_value= pwm_min;
                if(pwm_value>pwm_max) pwm_value= pwm_max;

                pwm_values[0]=pwm_value;
                pwm_values[1]=pwm_value;
                pwm_values[2]=pwm_value;

                pwm_values[3]=pwm_value;
                pwm_values[4]=pwm_value;
                pwm_values[5]=pwm_value;

                i_update_pwm.update_server_control_data(
                        pwm_values[0],  pwm_values[1],  pwm_values[2],
                        pwm_values[3],  pwm_values[4],  pwm_values[5],
                        pwm_on,  safe_torque_off_mode);
            }

            time +=sync_inc;
            break;
        }
    }
}


int main(void) {

    interface WatchdogInterface i_watchdog[2];
    interface update_pwm_general i_update_pwm;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[2];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        on tile[IFM_TILE]:
        {
            par
            {
                /* PWM Service */
                {
                    pwm_config_general(pwm_ports, REF_CLK_FRQ_MHZ_, PWM_CLK_FRQ_MHZ_);

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    pwm_service_general(
                            pwm_ports, i_update_pwm,
                            REF_CLK_FRQ_MHZ_, PWM_CLK_FRQ_MHZ_, COMMUTATION_FRQ_KHZ_);

                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, REF_CLK_FRQ_MHZ_);//20ms
                }

                /* PWM CLIENT */
                {
                    send_pwm_values(
                            i_update_pwm,
                            i_watchdog[0],
                            REF_CLK_FRQ_MHZ_,
                            PWM_CLK_FRQ_MHZ_,
                            COMMUTATION_FRQ_KHZ_);
                }
            }
        }
    }

    return 0;
}

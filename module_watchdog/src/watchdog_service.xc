/**
 * @file watchdog.xc
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>
#include <watchdog_service.h>
#include <xs1.h>
#include <stdio.h>
#include <timer.h>
#include <xscope.h>

[[combinable]]
void watchdog_service(WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[4])
{
   WatchdogState state;
   unsigned timeout_enabled = 0;

       //Set freq to 250MHz (always needed for proper time calculation)
       //write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz


       //adc_stuff
       unsigned time_stamp; // Time stamp
       unsigned inp_val = 0, tmp_val = 0;
       int out_a = 0, out_b= 0;
       unsigned config = 0;
       int adc_data[2][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0}};

       set_thread_fast_mode_on();

       //configure_adc_ports_7265( adc_ports.p32_data , adc_ports.xclk, adc_ports.p1_serial_clk, adc_ports.p1_ready , adc_ports.p4_mux ); // Configure all ADC data ports



       //Watchdog

       unsigned using_shared=0;



       unsigned tick_out = 0;
       unsigned shared_out = (1 << 2); //IMPORTANT DIAG EN SIGNAL!!!!


       timer tmr_keep_alive;
       timer tmr_pulse;

       unsigned time_pulse;
       unsigned time_keep_alive;

       //Init timers
       tmr_pulse :> time_pulse;
       tmr_keep_alive :> time_keep_alive;

       //Init ports
       if(!isnull(watchdog_ports.p_shared_leds_wden))
       {
           using_shared=1;
           watchdog_ports.p_shared_leds_wden<:shared_out;
       }
       else if(!isnull(watchdog_ports.p_wd_enable) && !isnull(watchdog_ports.p_wd_tick))
       {
           watchdog_ports.p_wd_enable<:1;
       }
       else
       {
           printf("ERROR WATCHDOG - Either you define ports for wd_tick and wd_enable or you define the shared port!");
           while(1);
       }

       while(1)
       {
           select
           {
               /********************
                * INCOMING COMMAND *
                ********************/

               case i_watchdog[int i].start():
                       state=WD_STATE_EN;
                       break;


               case i_watchdog[int i].stop():
                       state=WD_STATE_OFF;
                       break;


               case i_watchdog[int i].enableKeepAliveTimeout():
                       timeout_enabled=1;
                       break;

               case i_watchdog[int i].disableKeepAliveTimeout():
                       timeout_enabled=0;
                       break;


               case  i_watchdog[int i].getState() -> WatchdogState o:
                       o=state;
                       break;


               case i_watchdog[int i].keepalive():
                       state=WD_STATE_EN;
                       tmr_keep_alive :> time_keep_alive;
                       break;

               /**********************
                * KEEP ALIVE TIMEOUT *
                **********************/
                case tmr_keep_alive when timerafter(time_keep_alive+watchdog_ports.keep_alive_ticks) :> time_keep_alive:
                        if(timeout_enabled) state=WD_STATE_OFF;
                break;

                /********************
                 * PULSE GENERATION *
                 ********************/
                case tmr_pulse when timerafter(time_pulse + 250000) :> time_pulse:

                    //WATCHDOG
                    if (state != WD_STATE_OFF)
                    {
                        if (using_shared){
                            shared_out ^= (1 << watchdog_ports.shared_pin_wd_tick)|(1 << watchdog_ports.shared_pin_wd_enable);
                            watchdog_ports.p_shared_leds_wden <: shared_out;
                        }
                        else
                        {
                            tick_out ^= 0x1;
                            watchdog_ports.p_wd_tick <: tick_out;
                            watchdog_ports.p_wd_enable <: tick_out;
                        }
                    }

                   break;

           }//SELECT END
       }//WHILE END
}

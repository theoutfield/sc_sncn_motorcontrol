/**
 * @file watchdog.xc
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>
#include <watchdog_service.h>

[[combinable]]
 void watchdog_service(WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[2])
{

    //Set freq to 250MHz (always needed for proper time calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    unsigned char p_led_motoon_wdtick_wden_buffer = 0b1000;
    unsigned char reset_wd_en_mask = 0b1110;
    unsigned char   set_wd_en_mask = 0b0001;

    unsigned char p_ifm_wdtick = 0b0000;
    unsigned char reset_wd_tick_mask = 0b0000;
    unsigned char   set_wd_tick_mask = 0b0001;

    unsigned char reset_led_mask = 0b0111;
    unsigned char   set_led_mask = 0b1000;

    unsigned char reset_motoon_mask = 0b1011;
    unsigned char   set_motoon_mask = 0b0100;

    unsigned char   fault_mask = 0b1000;

    int initialization  =0;
    int WD_En_sent_flag =0;
    unsigned int wd_enabled = 0;
    unsigned int ts;
    timer t;

    unsigned int LED_counter = 0;
    int fault=0;//FIXME: this variable should be initialized to 0. here it is 3 to check the LED flashing of WD task
    int fault_counter=0;

    if (initialization == 0)
    {
        //motor on
        p_led_motoon_wdtick_wden_buffer |= set_motoon_mask;
        watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

        //reset WD_EN and LED
        p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
        p_led_motoon_wdtick_wden_buffer &= reset_wd_en_mask;

        watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

        //Enable WD
        p_led_motoon_wdtick_wden_buffer |= set_wd_en_mask;
        watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

        initialization = 1;
        wd_enabled = 1;

        t :> ts;
        t when timerafter (ts + 25000  ) :> void;
    }


    t :> ts;

    // Loop forever processing commands
    while (1) {
        select {
            // Get a command from the out loop
        case i_watchdog[int i].start(): // produce a rising edge on the WD_EN
                wd_enabled = 1;
                break;

        case i_watchdog[int i].stop():
                // Disable the kicking
                wd_enabled = 0;
                break;

        case i_watchdog[int i].protect(int fault_id):
                p_led_motoon_wdtick_wden_buffer &= fault_mask;
                watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

                if (!isnull(watchdog_ports.p_tick))
                {
                    p_ifm_wdtick &= reset_wd_tick_mask;
                    watchdog_ports.p_tick <: p_ifm_wdtick;
                }

                fault=fault_id;
                wd_enabled = 0;
                break;

        case t when timerafter(ts + 5000) :> void: // 5000 is equal to 20 us when reference frequency is 250 MHz

                t :> ts;
                if (initialization == 1)
                {
                    if (wd_enabled == 1)
                    {

                        if (!isnull(watchdog_ports.p_tick))
                        {
                            if ((p_ifm_wdtick & set_wd_tick_mask) == 0)
                                p_ifm_wdtick |= set_wd_tick_mask;
                            else
                                p_ifm_wdtick &= reset_wd_tick_mask;

                            watchdog_ports.p_tick <: p_ifm_wdtick;
                        }
                        else
                        {
                            if ((p_led_motoon_wdtick_wden_buffer & 0b0010) == 0)
                                p_led_motoon_wdtick_wden_buffer |= 0b0010;
                            else
                                p_led_motoon_wdtick_wden_buffer &= 0b1101;

                            watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;
                        }


                        if (WD_En_sent_flag<2)
                        {
                            if ((p_led_motoon_wdtick_wden_buffer & set_wd_en_mask) == 0)
                                p_led_motoon_wdtick_wden_buffer |= set_wd_en_mask;
                            else
                                p_led_motoon_wdtick_wden_buffer &= reset_wd_en_mask;

                            watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

                            WD_En_sent_flag++;
                        }
                    }

                    LED_counter++;
                    if (LED_counter >= 15000)
                    {
                        LED_counter=0;
                        fault_counter++;
                        if(fault==0)
                        {
                            if ((p_led_motoon_wdtick_wden_buffer & set_led_mask) == 0)
                                p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            else
                                p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            LED_counter=14000;
                        }
                        //showing the fault type by LED flashing (once, twice, ..., five times)
                        if(fault==1)
                        {
                            if(!isnull(watchdog_ports.p_tick))
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            }
                            else
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            }
                        }
                        if(fault==2)
                        {
                            if(!isnull(watchdog_ports.p_tick))
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            }
                            else
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            }

                        }
                        if(fault==3)
                        {
                            if(!isnull(watchdog_ports.p_tick))
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            }
                            else
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            }
                        }
                        if(fault==4)
                        {
                            if(!isnull(watchdog_ports.p_tick))
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==12)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==13)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            }
                            else
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==12)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==13)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            }
                        }
                        if(fault==5)
                        {
                            if(!isnull(watchdog_ports.p_tick))
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==12)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==13)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==14)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==15)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                            }
                            else
                            {
                                if(fault_counter== 5)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 6)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 7)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter== 8)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter== 9)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==10)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==11)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==12)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==13)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                                if(fault_counter==14)   p_led_motoon_wdtick_wden_buffer |= set_led_mask;
                                if(fault_counter==15)   p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                            }
                        }
                        watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;
                        if(fault_counter==20) fault_counter=0;
                    }
                }



                break;

        case i_watchdog[int i].reset_faults():

                p_led_motoon_wdtick_wden_buffer = 0b1000;
                p_ifm_wdtick = 0b0000;
                initialization  =0;
                WD_En_sent_flag =0;
                wd_enabled = 0;
                LED_counter = 0;
                fault=0;
                fault_counter=0;

                //motor on
                p_led_motoon_wdtick_wden_buffer |= set_motoon_mask;
                watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

                //reset WD_EN and LED
                p_led_motoon_wdtick_wden_buffer &= reset_led_mask;
                p_led_motoon_wdtick_wden_buffer &= reset_wd_en_mask;

                watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

                //Enable WD
                p_led_motoon_wdtick_wden_buffer |= set_wd_en_mask;
                watchdog_ports.p_enable <: p_led_motoon_wdtick_wden_buffer;

                initialization = 1;
                wd_enabled = 1;
                t :> ts;
                t when timerafter (ts + 25000  ) :> void;
                t :> ts;
                break;
        }
    }
}

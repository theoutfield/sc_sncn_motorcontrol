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

    unsigned int wd_enabled = 1, shared_out, tick_out = 0;
    unsigned int ts, ts2;
    timer t;


    unsigned char p_ifm_led_moton_wdtick_wden_buffer = 0xC;
    int initialization  =0;
    int WD_En_sent_flag =0;

    unsigned int LED_counter = 0;
    int fault=0;//FIXME: this variable should be initialized to 0. here it is 3 to check the LED flashing of WD task
    int fault_counter=0;


    t :> ts;

    // Loop forever processing commands
    while (1) {
        select {
            // Get a command from the out loop
                case i_watchdog[int i].start(): // produce a rising edge on the WD_EN
/*

                    if (isnull(watchdog_ports.p_tick)){
                        shared_out = 0xe;
                        shared_out &= ~0x5;//and shortly switch the LED to red (DC1K)
                    }
                    else {
                        shared_out &= ~0x1;
                    }
                    watchdog_ports.p_enable <: shared_out; // go low

                    t :> ts2;
                    t when timerafter(ts2+25000) :> ts2; // FIXME Magic numbers. Magic numbers everywhere...

                    if (isnull(watchdog_ports.p_tick)){
                        shared_out &= 0x7;
                        shared_out |= 0x1;
                    }
                    else{
                        shared_out |= 0x1;
                    }
                    watchdog_ports.p_enable <: shared_out; // go high

                    t :> ts2;
                    t when timerafter (ts2 + 25000) :> ts2; // FIXME Magic numbers. Magic numbers everywhere...

                    if (isnull(watchdog_ports.p_tick)){
                        shared_out |= (1 << 2);             // prepare the kicking
                        watchdog_ports.p_enable <: shared_out;
                    }

                    wd_enabled = 1;
*/
                    wd_enabled = 1;
                    break;

                case i_watchdog[int i].stop():
                    // Disable the kicking
                    wd_enabled = 0;
                    break;

                case i_watchdog[int i].protect(int fault_id):
                    p_ifm_led_moton_wdtick_wden_buffer &= 0b1011; // disabling the "motor enable" pin
                    watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;

                    fault=fault_id;
                    wd_enabled = 0;
                    break;

                case t when timerafter(ts + 500) :> void: // 500 is equal to 2 us when reference frequency is 250 MHz

                    t :> ts;
                    if (initialization == 0)
                    {
                        //motor on
                        p_ifm_led_moton_wdtick_wden_buffer |= 0b0100;
                        watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;


                        //reset WD_EN, WD_TICK and LED
                        p_ifm_led_moton_wdtick_wden_buffer &= 0b0100;
                        watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;

                        //Enable WD
                        p_ifm_led_moton_wdtick_wden_buffer |= 0b0001;
                        watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;

                        initialization = 1;
                        wd_enabled = 1;
                    }

                    if (initialization == 1)
                    {
                        if (wd_enabled == 1)
                        {

                            // toggling WD_EN
                            if ((p_ifm_led_moton_wdtick_wden_buffer & 0b0010) == 0)
                                p_ifm_led_moton_wdtick_wden_buffer |= 0b0010;
                            else
                                p_ifm_led_moton_wdtick_wden_buffer &= 0b1101;

                            watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;


                            if (WD_En_sent_flag<2)
                            {
                                //Toggle WD_EN
                                if ((p_ifm_led_moton_wdtick_wden_buffer & 0b0001) == 0)
                                    p_ifm_led_moton_wdtick_wden_buffer |= 0b0001;
                                else
                                    p_ifm_led_moton_wdtick_wden_buffer &= 0b1110;
                                watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;

                                WD_En_sent_flag++;
                            }

                        }

                        LED_counter++;
                        if (LED_counter >= 150000)
                        {
                            fault_counter++;


                            if(fault==0)//LED permanently on
                            {
                                p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                            }

                            //showing the fault type by LED flashing (once, twice, ..., five times)
                            if(fault==1)
                            {
                                if(fault_counter== 5)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 6)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 7)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                            }

                            if(fault==2)
                            {
                                if(fault_counter== 5)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 6)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 7)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 8)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 9)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                            }

                            if(fault==3)
                            {
                                if(fault_counter== 5)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 6)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 7)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 8)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 9)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==10)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==11)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                            }

                            if(fault==4)
                            {
                                if(fault_counter== 5)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 6)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 7)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 8)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 9)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==10)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==11)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==12)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==13)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                            }

                            if(fault==5)
                            {
                                if(fault_counter== 5)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 6)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 7)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter== 8)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter== 9)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==10)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==11)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==12)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==13)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                                if(fault_counter==14)   p_ifm_led_moton_wdtick_wden_buffer |= 0b1000;
                                if(fault_counter==15)   p_ifm_led_moton_wdtick_wden_buffer &= 0b0111;
                            }

                            watchdog_ports.p_enable <: p_ifm_led_moton_wdtick_wden_buffer;

                            if(fault_counter==20) fault_counter=0;
                            LED_counter = 0;

                        }

/*
                    if (isnull(watchdog_ports.p_tick)){
                        shared_out ^= (1 << 1) ; //toggle the second bit (DC1K)
                        watchdog_ports.p_enable <: shared_out;
                    }
                    else{
                        tick_out ^= 0x1;
                        watchdog_ports.p_tick <: tick_out;
                    }
*/
                    }
                    break;

            // Do a case: Separate the LED bits from the port XS1_PORT_4B. LSB enable watchdog.
        }
    }
}

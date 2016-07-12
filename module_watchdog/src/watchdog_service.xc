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

    unsigned char p_ledblue_ledgreen_ledred_wden = 0b1110;
    unsigned char reset_wd_en_mask = 0b1110;
    unsigned char   set_wd_en_mask = 0b0001;

    unsigned char p_ifm_wdtick = 0b0000;
    unsigned char reset_wd_tick_mask = 0b0000;
    unsigned char   set_wd_tick_mask = 0b0001;

    unsigned char reset_ledred_mask = 0b1101;
    unsigned char   set_ledred_mask = 0b0010;

    unsigned char reset_ledgreen_mask = 0b1011;
    unsigned char   set_ledgreen_mask = 0b0100;

    int initialization  =0;
    int WD_En_sent_flag =0;
    unsigned int wd_enabled = 0;
    unsigned int ts;
    timer t;

    unsigned int LED_counter = 0;
    int fault=5;//FIXME: this variable should be initialized to 0. here it is 3 to check the LED flashing of WD task
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
                //                p_ledblue_ledgreen_ledred_wden &= 0b1011; // disabling the "motor enable" pin
                //                watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;
                //
                //                fault=fault_id;
                //                wd_enabled = 0;
                break;

        case t when timerafter(ts + 500) :> void: // 500 is equal to 2 us when reference frequency is 250 MHz

                t :> ts;

                if (initialization == 1)
                {
                    if (wd_enabled == 1)
                    {
                        // Toggling WD_Tick
                        if ((p_ifm_wdtick & set_wd_tick_mask) == 0)
                            p_ifm_wdtick |= set_wd_tick_mask;
                        else
                            p_ifm_wdtick &= reset_wd_tick_mask;

                        watchdog_ports.p_tick <: p_ifm_wdtick;


                        if (WD_En_sent_flag<2)
                        {
                            // Toggling WD_EN
                            if ((p_ledblue_ledgreen_ledred_wden & set_wd_en_mask) == 0)
                                p_ledblue_ledgreen_ledred_wden |= set_wd_en_mask;
                            else
                                p_ledblue_ledgreen_ledred_wden &= reset_wd_en_mask;

                            watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;

                            WD_En_sent_flag++;
                        }

                    }

                    LED_counter++;
                    if (LED_counter >= 150000)
                    {
                        fault_counter++;

                        if(fault==0)//GREEN LED BLINKING
                        {
                            if ((p_ledblue_ledgreen_ledred_wden & set_ledgreen_mask) == 0)
                                p_ledblue_ledgreen_ledred_wden |= set_ledgreen_mask;
                            else
                                p_ledblue_ledgreen_ledred_wden &= reset_ledgreen_mask;
                        }


                        //showing the fault type by LED flashing (once, twice, ..., five times)
                        if(fault==1)
                        {
                            if(fault_counter== 5)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 6)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 7)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                        }

                        if(fault==2)
                        {
                            if(fault_counter== 5)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 6)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 7)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 8)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 9)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                        }

                        if(fault==3)
                        {
                            if(fault_counter== 5)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 6)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 7)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 8)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 9)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==10)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==11)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                        }

                        if(fault==4)
                        {
                            if(fault_counter== 5)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 6)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 7)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 8)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 9)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==10)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==11)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==12)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==13)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                        }

                        if(fault==5)
                        {
                            if(fault_counter== 5)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 6)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 7)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter== 8)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter== 9)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==10)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==11)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==12)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==13)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                            if(fault_counter==14)   p_ledblue_ledgreen_ledred_wden &= reset_ledred_mask;
                            if(fault_counter==15)   p_ledblue_ledgreen_ledred_wden |= set_ledred_mask;
                        }

                        watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;

                        if(fault_counter==20) fault_counter=0;
                        LED_counter = 0;

                    }

                }

                if (initialization == 0)
                {
//                    //motor on
//                    p_ledblue_ledgreen_ledred_wden |= 0b0100;
//                    watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;


                    //reset WD_EN and LED
                    p_ledblue_ledgreen_ledred_wden &= 0b0000;
                    p_ledblue_ledgreen_ledred_wden |= 0b1110;
                    watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;

                    //Enable WD
                    p_ledblue_ledgreen_ledred_wden |= set_wd_en_mask;
                    watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;

                    initialization = 1;
                    wd_enabled = 1;
                }

                break;

        case i_watchdog[int i].reset_faults():
                //
                //                p_ledblue_ledgreen_ledred_wden = 0xC;
                //
                //                LED_counter = 0;
                //                fault_counter=0;
                //                WD_En_sent_flag =0;
                //                fault=0;
                //
                //                t :> ts;
                //                //motor on
                //                p_ledblue_ledgreen_ledred_wden |= 0b0100;
                //                watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;
                //
                //                //reset WD_EN, WD_TICK and LED
                //                p_ledblue_ledgreen_ledred_wden &= 0b0100;
                //                watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;
                //
                //                //Enable WD
                //                p_ledblue_ledgreen_ledred_wden |= 0b0001;
                //                watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;
                //
                //                initialization = 1;
                //                wd_enabled = 1;
                //
                //                // toggling WD_EN
                //                if ((p_ledblue_ledgreen_ledred_wden & 0b0010) == 0)
                //                    p_ledblue_ledgreen_ledred_wden |= 0b0010;
                //                else
                //                    p_ledblue_ledgreen_ledred_wden &= 0b1101;
                //                watchdog_ports.p_enable <: p_ledblue_ledgreen_ledred_wden;
                //
                //                WD_En_sent_flag++;
                break;

                // Do a case: Separate the LED bits from the port XS1_PORT_4B. LSB enable watchdog.
        }
    }
}

/**
 * @file watchdog.xc
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>
#include <watchdog_service.h>

[[combinable]]
 void watchdog_service(WatchdogPorts &watchdog_ports, interface WatchdogInterface server i_watchdog[2], int ifm_tile_usec)
{
    unsigned int usec;
    if(ifm_tile_usec==250)
    {
        //Set freq to 250MHz (always needed for proper timing)
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
        usec = 250;
    }
    else
    {
        usec = 100;
    }

    int IFM_module_type = -1;

    unsigned int wd_half_period = 40 * usec;

    unsigned char led_motor_on_wdtick_wden_buffer = 0;
    unsigned char   set_wd_en_mask = 0b0001;
    unsigned char p_ifm_wdtick = 0b0000;
    //CPLD
    unsigned cpld_out_state = 0x8;//set green LED off
    unsigned int cycles_counter = 0;

    int WD_En_sent_flag =0;
    unsigned int wd_enabled = 0;
    unsigned int ts;
    timer t;

    unsigned int LED_counter = 0;
    int fault=0;//FIXME: this variable should be initialized to 0. here it is 3 to check the LED flashing of WD task
    int fault_counter=0;
    unsigned int times = 0;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (1000*20*250)) :> void;//FixMe: how is it proper?

    //Do the IFM type identification only once
    if(!isnull(watchdog_ports.p_shared_enable_tick_led)){//DC100, DC300, or DC1K
        if(!isnull(watchdog_ports.p_tick)){
            IFM_module_type = DC100_DC300;
        } else {
            IFM_module_type = DC1K_DC5K;
        }
    }
    else if(!isnull(watchdog_ports.p_cpld_shared)){//DC500
        IFM_module_type = DC500;
    }

    /* WD Initialization routine */
    switch(IFM_module_type){
        case DC100_DC300:
            //Enable WD
            led_motor_on_wdtick_wden_buffer |= set_wd_en_mask;
            //set green LED on
            led_motor_on_wdtick_wden_buffer |= 0b1010;
            watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
            wd_enabled = 1;
            break;
        case DC500:
            cpld_out_state &= 0b0111;//turn green LED on
            cpld_out_state |= 0b0101;//enable CPLD, reset errors
            watchdog_ports.p_cpld_shared <: cpld_out_state & 0xf;
            cycles_counter = 40;
            wd_enabled = 1;
            break;
        case DC1K_DC5K://FixMe: optimize it further
            //motor on and WD on
            led_motor_on_wdtick_wden_buffer |= 0b0101;//[ LED | Motor_En | WD_Tick | WD_En ]
            wd_enabled = 1;
            break;
    }

    t :> ts;
    t when timerafter (ts + 100*usec) :> void;//100 us


    t :> ts;

    // Loop forever processing commands
    while (1) {
        select {
        case i_watchdog[int i].status() -> {int status}:
                status = ACTIVE;
                break;

                //FixMe: this call is absolete, should be removed
        case i_watchdog[int i].start(): // produce a rising edge on the WD_EN
                wd_enabled = 1;
                break;

                //FixMe: this call is absolete, should be removed
        case i_watchdog[int i].stop():
                // Disable the kicking
                wd_enabled = 0;
                break;

        case i_watchdog[int i].protect(int fault_id):
                switch(IFM_module_type){
                    case DC100_DC300:
                        //Disable WD and set red LED on
                        led_motor_on_wdtick_wden_buffer &= 0b1100;
                        watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
                        break;
                    case DC500:
                        cpld_out_state |= 0b1000;//set green LED off
                        watchdog_ports.p_cpld_shared <: cpld_out_state & 0xf;
                        break;
                    case DC1K_DC5K://[ LED | Motor_En | WD_Tick | WD_En ]
                        led_motor_on_wdtick_wden_buffer &= 0b1000;
                        watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
                        break;
                }

                fault=fault_id;
                wd_enabled = 0;
                break;

        case t when timerafter(ts + wd_half_period) :> void://clocking

                t :> ts;
                if (wd_enabled == 1)
                {
                    switch(IFM_module_type){
                        case DC100_DC300:
                            p_ifm_wdtick ^= 1;//toggle wd tick
                            watchdog_ports.p_tick <: p_ifm_wdtick;
                            break;
                        case DC500:
                            cpld_out_state ^= (1 << 1); //toggle wd tick
                            watchdog_ports.p_cpld_shared <: cpld_out_state & 0xf;

                            //keep fault reset pin high for some number of cycles to charge the cap
                            if(cycles_counter > 0){
                                cycles_counter--;
                                if(!cycles_counter){//drive reset pin down
                                    cpld_out_state &= 0b1110;
                                    watchdog_ports.p_cpld_shared <: cpld_out_state;
                                }
                            }
                            break;
                        case DC1K_DC5K: //[ LED | Motor_En | WD_Tick | WD_En ]
                            led_motor_on_wdtick_wden_buffer ^= (1 << 1); //toggle wd tick
                            watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
                            //Reset WD after fault
                            if (WD_En_sent_flag<4)
                            {
                                if(WD_En_sent_flag % 2 == 0){
                                    led_motor_on_wdtick_wden_buffer ^= 1;//toggle WD Enable pin
                                    watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
                                }
                                WD_En_sent_flag++;
                            }
                            break;
                    }
                }

                //showing the fault type by LED flashing (once, twice, ..., five times)
                if(fault) blink_red(fault, 5000, watchdog_ports, IFM_module_type, led_motor_on_wdtick_wden_buffer, times, LED_counter);

                LED_counter++;

                break;

        case i_watchdog[int i].reset_faults():

                WD_En_sent_flag =0;
                LED_counter = 0;
                fault=0;
                fault_counter=0;

                switch(IFM_module_type){
                    case DC100_DC300://ToDo: this needs to be tested!
                        //reset WD_EN and LED
                        led_motor_on_wdtick_wden_buffer &= 0b0000;
                        //Turn green LED on and enable WD
                        led_motor_on_wdtick_wden_buffer |= 0b1011;
                        watchdog_ports.p_shared_enable_tick_led <: led_motor_on_wdtick_wden_buffer;
                        break;
                    case DC500:
                        cpld_out_state &= 0b0111;//turn green LED on
                        cpld_out_state |= 0b0101;//enable CPLD, reset errors
                        watchdog_ports.p_cpld_shared <: cpld_out_state & 0xf;
                        cycles_counter = 40;
                        break;
                    case DC1K_DC5K://FixMe: optimize the code. Why do we write 3 times?
                        //Reset all pins to zero, do not touch WD tick
                        led_motor_on_wdtick_wden_buffer &= 0b0010;//[ LED | Motor_En | WD_Tick | WD_En ]
                        //Set green LED on, enable WD
                        led_motor_on_wdtick_wden_buffer |= 0b0101;
                        break;
                }

                wd_enabled = 1;
                t :> ts;
                t when timerafter (ts + 100*usec  ) :> void;//100 us
                t :> ts;
                break;
        }
    }
}

void blink_red(int fault, int period, WatchdogPorts &watchdog_ports, int IFM_module_type, unsigned char &output, unsigned int &times, unsigned int &delay_counter){
    if ((delay_counter % period == 0) && times != (fault*2)){//blinking
        switch(IFM_module_type){
            case DC100_DC300://ToDo: to be tested
                output |= 0b1100;
                output ^= (1 << 1);
                watchdog_ports.p_shared_enable_tick_led <: output;
                break;
            case DC500://ToDo: to be tested
                output ^= (1 << 3);
                watchdog_ports.p_cpld_shared <: output;
                break;
            case DC1K_DC5K:
                output ^= (1 << 3);
                watchdog_ports.p_shared_enable_tick_led <: output;
                break;
            }
        times++;
    }
    else if ((delay_counter % (period*20) == 0) && times == (fault*2)){//idling
        times = 0;
    }
}

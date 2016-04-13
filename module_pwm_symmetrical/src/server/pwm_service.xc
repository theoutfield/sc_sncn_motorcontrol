/*
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include <xs1.h>

#include <pwm_service.h>
#include <refclk.h>

void disable_fets(PwmPorts &ports){

    ports.p_pwm[0] <: 0;
    ports.p_pwm[1] <: 0;
    ports.p_pwm[2] <: 0;

    ports.p_pwm_inv[0] <: 0;
    ports.p_pwm_inv[1] <: 0;
    ports.p_pwm_inv[2] <: 0;


    if(!isnull(ports.p_pwm_phase_d))
        ports.p_pwm_phase_d <: 0;
    if(!isnull(ports.p_pwm_phase_d_inv))
        ports.p_pwm_phase_d_inv <: 0;

    delay_milliseconds(1);
}

static void do_pwm_port_config_inv(  buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], clock clk )
{
    unsigned i;



    for (i = 0; i < PWM_CHAN_COUNT; i++)
    {
        configure_out_port(p_pwm[i], clk, 0);
        if (!isnull(p_pwm_inv)){
            configure_out_port(p_pwm_inv[i], clk, 0);
            set_port_inv(p_pwm_inv[i]);
        }
    }

    start_clock(clk);
}

static void do_pwm_port_config_inv_adc_trig( in port dummy, buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], clock clk )
{
    unsigned i;

    for (i = 0; i < PWM_CHAN_COUNT; i++)
    {
        configure_out_port(p_pwm[i], clk, 0);
        if (!isnull(p_pwm_inv)){
            configure_out_port(p_pwm_inv[i], clk, 0);
            set_port_inv(p_pwm_inv[i]);
        }
    }

    /* dummy port used to send ADC trigger */
    configure_in_port(dummy,clk);

    start_clock(clk);
}

void brake_service(buffered out port:32 p_pwm, buffered out port:32 p_pwm_inv, interface BrakeInterface server i_brake) {
    const unsigned delay = 5*USEC_FAST;
    const int duty = 32;
    const int period = 10;
    const int period_init = 100;
    const int init_duration = 100*MSEC_FAST;
    int brake_enable = 0;
    int brake_init = 0;
    timer t;
    unsigned int ts;
    unsigned int t_init;
    t :> ts;
    t_init = ts + delay;

    while(1) {
        select {
        case t when timerafter(ts + delay) :> ts:
            if (brake_enable) {
                if (brake_init == 0) {
                    //pwm release
                    p_pwm <: 0xffffffff;
                    delay_ticks(period*duty);
                    p_pwm <: 0x00000000;
                    delay_ticks(delay);
                    p_pwm_inv<: 0xffffffff;
                    delay_ticks(period*(100-duty) + 2*delay);
                    p_pwm_inv <: 0x00000000;
                } else {
                    if (timeafter(ts, t_init+init_duration))
                        brake_init = 0;
                    //pwm init
                    p_pwm <: 0xffffffff;
                    delay_ticks(period_init*100);
                    p_pwm <: 0x00000000;
                    delay_ticks(delay);
                    p_pwm_inv<: 0xffffffff;
                    delay_ticks(2*delay);
                    p_pwm_inv <: 0x00000000;
                }
                t :> ts;
            }
            break;

        case i_brake.set_brake(int enable):
            if (enable && brake_enable == 0) {
                brake_init = 1;
                t :> ts;
                t_init = ts + delay;
            }
            brake_enable = enable;
            break;

        case i_brake.get_brake() -> int out_brake:
            out_brake = brake_enable;
            break;
        }
    }
}

extern unsigned pwm_op_inv( unsigned buf, buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], chanend c, unsigned control );

void pwm_service( PwmPorts &ports, chanend ?c_pwm, interface BrakeInterface server ?i_brake)
{
    //Set Tile Ref Freq to 250MHz
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    disable_fets(ports);

    par {
        /* brake pwm */
        {
            if(!isnull(ports.p_pwm_phase_d) && !isnull(ports.p_pwm_phase_d_inv) && !isnull(i_brake)) {
                brake_service(ports.p_pwm_phase_d, ports.p_pwm_phase_d_inv, i_brake);
            }
        }

        /* motor pwm */
        {
            if (!isnull(c_pwm)) {
                unsigned buf, control;

                /* First read the shared memory buffer address from the client */
                c_pwm :> control;

                /* configure the ports */
                do_pwm_port_config_inv( ports.p_pwm, ports.p_pwm_inv, ports.clk);

                /* wait for initial update */
                c_pwm :> buf;

                while (1)
                {
                    buf = pwm_op_inv( buf, ports.p_pwm, ports.p_pwm_inv, c_pwm, control );
                }
            }
        }
    }
}

extern unsigned pwm_op_inv_trig( unsigned buf, buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], chanend c, unsigned control, chanend c_trig, in port dummy_port );

void pwm_triggered_service(PwmPorts &ports, chanend c_adc_trig, chanend c_pwm, interface BrakeInterface server ?i_brake)
{
    //Set Tile Ref Freq to 250MHz
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    disable_fets(ports);

    par {
        /* brake pwm */
        {
            if(!isnull(ports.p_pwm_phase_d) && !isnull(ports.p_pwm_phase_d_inv) && !isnull(i_brake)) {
                brake_service(ports.p_pwm_phase_d, ports.p_pwm_phase_d_inv, i_brake);
            }
        }

        /* motor pwm */
        {
            unsigned buf, control;

            /* First read the shared memory buffer address from the client */
            c_pwm :> control;

            /* configure the ports */
            do_pwm_port_config_inv_adc_trig( ports.dummy_port, ports.p_pwm, ports.p_pwm_inv, ports.clk );

            /* wait for initial update */
            c_pwm :> buf;

            while (1)
            {
                buf = pwm_op_inv_trig( buf, ports.p_pwm, ports.p_pwm_inv, c_pwm, control, c_adc_trig, ports.dummy_port );
            }
        }
    }
}

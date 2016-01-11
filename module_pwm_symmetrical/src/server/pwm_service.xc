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

extern unsigned pwm_op_inv( unsigned buf, buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], chanend c, unsigned control );

void pwm_service( PwmPorts &ports, chanend c_pwm)
{
    //Set Tile Ref Freq to 250MHz
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    disable_fets(ports);

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

extern unsigned pwm_op_inv_trig( unsigned buf, buffered out port:32 p_pwm[], buffered out port:32 (&?p_pwm_inv)[], chanend c, unsigned control, chanend c_trig, in port dummy_port );

void pwm_triggered_service(PwmPorts &ports, chanend c_adc_trig, chanend c_pwm)
{

    //Set Tile Ref Freq to 250MHz
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    disable_fets(ports);

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

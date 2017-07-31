/*
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "pwm_server.h"
#include <pwm_ports.h>
#include <motor_control_interfaces.h>

///**
// * @brief Initialize the predriver circuit in your IFM SOMANET device (if applicable)
// *
// * @param fet_driver_ports  Structure of ports to manage the FET-driver in your IFM SOMANET device (if applicable).
// *
// * @return void
// */
//void predriver(FetDriverPorts &fet_driver_ports)
//{
//    const unsigned t_delay = 300*100;//300 us
//    unsigned int ts;
//    int check_fet;
//    int init_state = INIT_BUSY;
//    timer t;
//
//    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
//    {
//        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
//        t when timerafter (ts + t_delay) :> ts;
//    }
//
//    if(!isnull(fet_driver_ports.p_coast))
//    {
//        fet_driver_ports.p_coast :> check_fet;
//        init_state = check_fet;
//    }
//    else
//    {
//        init_state = 1;
//    }
//} // predriver

/**
 * @brief Configure the clock value and initial value of general pwm ports (with up to 6 inverter outputs).
 *
 * @param ports  Structure type for PWM ports
 *
 * @return void
 */
static void do_pwm_port_config_general(PwmPortsGeneral &ports)
{

    if (!isnull(ports.p_pwm_a))
    {
        configure_out_port( ports.p_pwm_a    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_a, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_a);
    }

    if (!isnull(ports.p_pwm_b))
    {
        configure_out_port( ports.p_pwm_b    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_b, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_b);
    }

    if (!isnull(ports.p_pwm_c))
    {
        configure_out_port( ports.p_pwm_c    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_c, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_c);
    }

    if (!isnull(ports.p_pwm_u))
    {
        configure_out_port( ports.p_pwm_u    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_u, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_u);
    }

    if (!isnull(ports.p_pwm_v))
    {
        configure_out_port( ports.p_pwm_v    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_v, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_v);
    }

    if (!isnull(ports.p_pwm_w))
    {
        configure_out_port( ports.p_pwm_w    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_w, ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_w);
    }

//    // Check of ADC synchronisation is being used
//    if (1==_LOCK_ADC_TO_PWM && (!isnull(ports.dummy_port)))
//    {
//        // Configure dummy input port used to send ADC synchronisation pulse
//        configure_in_port( ports.dummy_port, ports.clk );
//    }

} // do_pwm_port_config_general

/**
 * @brief Configure the pwm ports (all FETs open) before starting pwm service.
 *
 * @param ports  Structure type for general PWM ports (up to 6 inverter outputs)
 *
 * @return void
 */
void pwm_config_general(PwmPortsGeneral &ports)
{
    // Configure clock rate to PLATFORM_REFERENCE_MHZ/1 (100 MHz)
    configure_clock_rate( ports.clk ,100 ,1 );

    do_pwm_port_config_general(ports);

    start_clock( ports.clk ); // Start common PWM clock, once all ports configured

    if (!isnull(ports.p_pwm_a))
    {
        ports.p_pwm_a     <: 0;
        if (!isnull(ports.p_pwm_inv_a))  ports.p_pwm_inv_a <: 1;
    }

    if (!isnull(ports.p_pwm_b))
    {
        ports.p_pwm_b     <: 0;
        if (!isnull(ports.p_pwm_inv_b))  ports.p_pwm_inv_b <: 1;
    }

    if (!isnull(ports.p_pwm_c))
    {
        ports.p_pwm_c     <: 0;
        if (!isnull(ports.p_pwm_inv_c))  ports.p_pwm_inv_c <: 1;
    }

    if (!isnull(ports.p_pwm_u))
    {
        ports.p_pwm_u     <: 0;
        if (!isnull(ports.p_pwm_inv_u))  ports.p_pwm_inv_u <: 1;
    }

    if (!isnull(ports.p_pwm_v))
    {
        ports.p_pwm_v     <: 0;
        if (!isnull(ports.p_pwm_inv_v))  ports.p_pwm_inv_v <: 1;
    }

    if (!isnull(ports.p_pwm_w))
    {
        ports.p_pwm_w     <: 0;
        if (!isnull(ports.p_pwm_inv_w))  ports.p_pwm_inv_w <: 1;
    }

} // pwm_config_general

/**
 * @brief Service to generate center-alligned PWM signals for 6 inverter outputs (2 power switch for each leg).
 * It recieves 6 pwm values through i_update_pwm interface. The commutation frequency is 16 kHz, and the deadtime is 3 us.
 *
 * @param ports                 Structure type for PWM ports
 * @param i_update_pwm          Interface to communicate with client and update the PWM values
 *
 * @return void
 */
void pwm_service_general(
        PwmPortsGeneral &ports,
        server interface UpdatePWMGeneral i_update_pwm
)
{
    unsigned short phase_a_defined    =0x0000, phase_b_defined    =0x0000, phase_c_defined    =0x0000;
    unsigned short phase_a_inv_defined=0x0000, phase_b_inv_defined=0x0000, phase_c_inv_defined=0x0000;

    unsigned short phase_u_defined    =0x0000, phase_v_defined    =0x0000, phase_w_defined    =0x0000;
    unsigned short phase_u_inv_defined=0x0000, phase_v_inv_defined=0x0000, phase_w_inv_defined=0x0000;

    unsigned short pwm_value_a=0x0000, pwm_value_b=0x0000, pwm_value_c=0x0000, pwm_value_u=0x0000, pwm_value_v=0x0000, pwm_value_w=0x0000;

    unsigned short a_high_rise=0x0000, a_low_rise=0x0000;
    unsigned short b_high_rise=0x0000, b_low_rise=0x0000;
    unsigned short c_high_rise=0x0000, c_low_rise=0x0000;
    unsigned short u_high_rise=0x0000, u_low_rise=0x0000;
    unsigned short v_high_rise=0x0000, v_low_rise=0x0000;
    unsigned short w_high_rise=0x0000, w_low_rise=0x0000;

//    timer t;
//    unsigned int time    =0x00000000, ts   =0x00000000;

    unsigned int ref_time      = 0x00000000;

    unsigned int period_start  = 0x00000000;
    unsigned int period_middle = 0x00000000;
    unsigned int period_end    = 0x00000000;
    unsigned int dummy_value   = 0x00000000;

    unsigned inp_wid = 0x0000FFFF;
    unsigned pattern = 0x00000000;

    unsigned short port_clock_shift =0x0000;
    unsigned short period           =0x0000;
    unsigned short inactive_period  =0x0000;
    unsigned short pwm_limit_h      =0x0000;
    unsigned short pwm_limit_l      =0x0000;
    unsigned short limit_h_computational_margine = 0x0000;
    unsigned short limit_l_computational_margine = 0x0000;
    unsigned short pulse_generation_flag = 0x0000;

    unsigned short pwm_init =0x0000;

    // initialization:
    // ===============

    // values for 15 kHz switching frequency, and 100 MHz of ref_clck_frq:
    // ======================================
    port_clock_shift = 3333 & 0x0000FFFF   ;
    pwm_init         = 3333 & 0x0000FFFF   ;
    inactive_period  = (DEADTIME/10) & 0x0000FFFF   ;

    limit_h_computational_margine =220 & 0x0000FFFF   ;
    limit_l_computational_margine =220 & 0x0000FFFF   ;

    pwm_limit_h      = (6667 - (2*inactive_period) - limit_h_computational_margine) & 0x0000FFFF   ;
    pwm_limit_l      = (2*DEADTIME                 + limit_l_computational_margine) & 0x0000FFFF   ;



    phase_a_defined     = !isnull(ports.p_pwm_a);
    phase_a_inv_defined = !isnull(ports.p_pwm_inv_a);
    phase_b_defined     = !isnull(ports.p_pwm_b);
    phase_b_inv_defined = !isnull(ports.p_pwm_inv_b);
    phase_c_defined     = !isnull(ports.p_pwm_c);
    phase_c_inv_defined = !isnull(ports.p_pwm_inv_c);
    phase_u_defined     = !isnull(ports.p_pwm_u);
    phase_u_inv_defined = !isnull(ports.p_pwm_inv_u);
    phase_v_defined     = !isnull(ports.p_pwm_v);
    phase_v_inv_defined = !isnull(ports.p_pwm_inv_v);
    phase_w_defined     = !isnull(ports.p_pwm_w);
    phase_w_inv_defined = !isnull(ports.p_pwm_inv_w);

    pwm_value_a=pwm_init;
    pwm_value_b=pwm_init;
    pwm_value_c=pwm_init;
    pwm_value_u=pwm_init;
    pwm_value_v=pwm_init;
    pwm_value_w=pwm_init;

    a_high_rise= (pwm_init >> 1);
    a_low_rise = a_high_rise+inactive_period;
    b_high_rise= (pwm_init >> 1);
    b_low_rise = b_high_rise+inactive_period;
    c_high_rise= (pwm_init >> 1);
    c_low_rise = c_high_rise+inactive_period;
    u_high_rise= (pwm_init >> 1);
    u_low_rise = u_high_rise+inactive_period;
    v_high_rise= (pwm_init >> 1);
    v_low_rise = v_high_rise+inactive_period;
    w_high_rise= (pwm_init >> 1);
    w_low_rise = w_high_rise+inactive_period;

    while (1)
    {
        #pragma ordered
        select
        {
        case i_update_pwm.update_server_control_data(
                unsigned short pwm_a, unsigned short pwm_b, unsigned short pwm_c,
                unsigned short pwm_u, unsigned short pwm_v, unsigned short pwm_w,
                unsigned short pwm_on,unsigned short safe_torque_off):

                        pwm_value_a = (pwm_a & 0x0000FFFF);
                        if(pwm_value_a<pwm_limit_l) pwm_value_a=0x0000;
                        if(pwm_value_a>pwm_limit_h) pwm_value_a=pwm_limit_h;

                        pwm_value_b = (pwm_b & 0x0000FFFF);
                        if(pwm_value_b<pwm_limit_l) pwm_value_b=0x0000;
                        if(pwm_value_b>pwm_limit_h) pwm_value_b=pwm_limit_h;

                        pwm_value_c = (pwm_c & 0x0000FFFF);
                        if(pwm_value_c<pwm_limit_l) pwm_value_c=0x0000;
                        if(pwm_value_c>pwm_limit_h) pwm_value_c=pwm_limit_h;

                        pwm_value_u = (pwm_u & 0x0000FFFF);
                        if(pwm_value_u<pwm_limit_l) pwm_value_u=0x0000;
                        if(pwm_value_u>pwm_limit_h) pwm_value_u=pwm_limit_h;

                        pwm_value_v = (pwm_v & 0x0000FFFF);
                        if(pwm_value_v<pwm_limit_l) pwm_value_v=0x0000;
                        if(pwm_value_v>pwm_limit_h) pwm_value_v=pwm_limit_h;

                        pwm_value_w = (pwm_w & 0x0000FFFF);
                        if(pwm_value_w<pwm_limit_l) pwm_value_w=0x0000;
                        if(pwm_value_w>pwm_limit_h) pwm_value_w=pwm_limit_h;

                        a_high_rise= (pwm_value_a >> 1);
                        a_low_rise =  a_high_rise+inactive_period;
                        b_high_rise= (pwm_value_b >> 1);
                        b_low_rise =  b_high_rise+inactive_period;
                        c_high_rise= (pwm_value_c >> 1);
                        c_low_rise =  c_high_rise+inactive_period;
                        u_high_rise= (pwm_value_u >> 1);
                        u_low_rise =  u_high_rise+inactive_period;
                        v_high_rise= (pwm_value_v >> 1);
                        v_low_rise =  v_high_rise+inactive_period;
                        w_high_rise= (pwm_value_w >> 1);
                        w_low_rise =  w_high_rise+inactive_period;

                        pattern = peek( ports.p_pwm_a ); // Find out value on 1-bit port. NB Only LS-bit is relevant
                        ref_time  = partout_timestamped( ports.p_pwm_a ,1 ,pattern ); // Re-load output port with same bit-value
                        ref_time &= 0x0000FFFF;

                        ref_time += port_clock_shift;
                        ref_time &= 0x0000FFFF;

                        ref_time += 100;
                        ref_time &= 0x0000FFFF;

                        pulse_generation_flag = 1;

                        break;

        case i_update_pwm.status() -> {int status}:
                status = ACTIVE;
                break;

        case i_update_pwm.safe_torque_off_enabled():
            break;
        }//select

        if(pulse_generation_flag)
        {
            if(phase_a_inv_defined && pwm_value_a) ports.p_pwm_a           @ (unsigned short)((ref_time - a_high_rise)&(inp_wid)) <: 1;
            if(phase_a_inv_defined && pwm_value_a) ports.p_pwm_inv_a       @ (unsigned short)((ref_time - a_low_rise) &(inp_wid)) <: 1;
            if(phase_b_inv_defined && pwm_value_b) ports.p_pwm_b           @ (unsigned short)((ref_time - b_high_rise)&(inp_wid)) <: 1;
            if(phase_b_inv_defined && pwm_value_b) ports.p_pwm_inv_b       @ (unsigned short)((ref_time - b_low_rise) &(inp_wid)) <: 1;
            if(phase_c_inv_defined && pwm_value_c) ports.p_pwm_c           @ (unsigned short)((ref_time - c_high_rise)&(inp_wid)) <: 1;
            if(phase_c_inv_defined && pwm_value_c) ports.p_pwm_inv_c       @ (unsigned short)((ref_time - c_low_rise) &(inp_wid)) <: 1;
            if(phase_u_inv_defined && pwm_value_u) ports.p_pwm_u           @ (unsigned short)((ref_time - u_high_rise)&(inp_wid)) <: 1;
            if(phase_u_inv_defined && pwm_value_u) ports.p_pwm_inv_u       @ (unsigned short)((ref_time - u_low_rise) &(inp_wid)) <: 1;
            if(phase_v_inv_defined && pwm_value_v) ports.p_pwm_v           @ (unsigned short)((ref_time - v_high_rise)&(inp_wid)) <: 1;
            if(phase_v_inv_defined && pwm_value_v) ports.p_pwm_inv_v       @ (unsigned short)((ref_time - v_low_rise) &(inp_wid)) <: 1;
            if(phase_w_inv_defined && pwm_value_w) ports.p_pwm_w           @ (unsigned short)((ref_time - w_high_rise)&(inp_wid)) <: 1;
            if(phase_w_inv_defined && pwm_value_w) ports.p_pwm_inv_w       @ (unsigned short)((ref_time - w_low_rise) &(inp_wid)) <: 1;

            if(phase_a_inv_defined && pwm_value_a) ports.p_pwm_a           @ (unsigned short)((ref_time + a_high_rise)&(inp_wid)) <: 0;
            if(phase_a_inv_defined && pwm_value_a) ports.p_pwm_inv_a       @ (unsigned short)((ref_time + a_low_rise) &(inp_wid)) <: 0;
            if(phase_b_inv_defined && pwm_value_b) ports.p_pwm_b           @ (unsigned short)((ref_time + b_high_rise)&(inp_wid)) <: 0;
            if(phase_b_inv_defined && pwm_value_b) ports.p_pwm_inv_b       @ (unsigned short)((ref_time + b_low_rise) &(inp_wid)) <: 0;
            if(phase_c_inv_defined && pwm_value_c) ports.p_pwm_c           @ (unsigned short)((ref_time + c_high_rise)&(inp_wid)) <: 0;
            if(phase_c_inv_defined && pwm_value_c) ports.p_pwm_inv_c       @ (unsigned short)((ref_time + c_low_rise) &(inp_wid)) <: 0;
            if(phase_u_inv_defined && pwm_value_u) ports.p_pwm_u           @ (unsigned short)((ref_time + u_high_rise)&(inp_wid)) <: 0;
            if(phase_u_inv_defined && pwm_value_u) ports.p_pwm_inv_u       @ (unsigned short)((ref_time + u_low_rise) &(inp_wid)) <: 0;
            if(phase_v_inv_defined && pwm_value_v) ports.p_pwm_v           @ (unsigned short)((ref_time + v_high_rise)&(inp_wid)) <: 0;
            if(phase_v_inv_defined && pwm_value_v) ports.p_pwm_inv_v       @ (unsigned short)((ref_time + v_low_rise) &(inp_wid)) <: 0;
            if(phase_w_inv_defined && pwm_value_w) ports.p_pwm_w           @ (unsigned short)((ref_time + w_high_rise)&(inp_wid)) <: 0;
            if(phase_w_inv_defined && pwm_value_w) ports.p_pwm_inv_w       @ (unsigned short)((ref_time + w_low_rise) &(inp_wid)) <: 0;

            pulse_generation_flag = 0;
        }// if(pulse_generation_flag)

    } // while(1)

} // pwm_service_general

//
///**
// * @brief Configure the clock value and initial value of pwm ports.
// *
// * @param ports  Structure type for PWM ports
// *
// * @return void
// */
//static void do_pwm_port_config(PwmPorts &ports)
//{
//    unsigned i;
//
//    // Loop through PWM phases
//    for (i = 0; i < _NUM_PWM_PHASES; i++)
//    {   // Configure ports for this phase
//
//        configure_out_port( ports.p_pwm[i] , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
//        configure_out_port( ports.p_pwm_inv[i] , ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
//        set_port_inv( ports.p_pwm_inv[i] );
//    }
//
//    if (!isnull(ports.p_pwm_phase_d))
//    {
//        configure_out_port( ports.p_pwm_phase_d , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
//        configure_out_port( ports.p_pwm_phase_d_inv , ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
//        set_port_inv( ports.p_pwm_phase_d_inv );
//    }
//
//    // Check of ADC synchronisation is being used
//    if (1 == _LOCK_ADC_TO_PWM)
//    {   // ADC synchronisation activated
//
//        // Configure dummy input port used to send ADC synchronisation pulse
//        configure_in_port( ports.dummy_port, ports.clk );
//    } // if (1 == LOCK_ADC_TO_PWM)
//
//} // do_pwm_port_config
//
///**
// * @brief Configure the pwm ports before starting pwm service.
// *
// * @param ports  Structure type for PWM ports
// *
// * @return void
// */
//void pwm_config(PwmPorts &ports)
//{
//
//    do_pwm_port_config(ports);
//
//    start_clock( ports.clk ); // Start common PWM clock, once all ports configured
//
//    ports.p_pwm[_PWM_PHASE_A]  <: 0x00000000;
//    ports.p_pwm_inv[_PWM_PHASE_A]  <: 0xFFFFFFFF;
//
//    ports.p_pwm[_PWM_PHASE_B]  <: 0x00000000;
//    ports.p_pwm_inv[_PWM_PHASE_B]  <: 0xFFFFFFFF;
//
//    ports.p_pwm[_PWM_PHASE_C]  <: 0x00000000;
//    ports.p_pwm_inv[_PWM_PHASE_C]  <: 0xFFFFFFFF;
//
//    if (!isnull(ports.p_pwm_phase_d))
//    {
//        ports.p_pwm_phase_d  <: 0x00000000;
//        ports.p_pwm_phase_d_inv  <: 0xFFFFFFFF;
//    }
//} // pwm_config
//
///**
// * @brief Service to generate center-alligned PWM signals for 3 inverter outputs.
// * it also provides PWM signals to turn on/off an electric brake.
// *
// * @param motor_id              Motor ID (the default value is 0)
// * @param ports                 Structure type for PWM ports
// * @param i_update_pwm          Interface to communicate with client and update the PWM values
// * @param duty_start_brake      PWM value which is used to start the electric brake
// * @param duty_maintain_brake   PWM value which is used to maintain the electric brake
// * @param time_start_brake      Required time to start the brake (in milliseconds)
// * @param ifm_tile_usec         Reference clock frequency of IFM tile (in MHz)
// *
// * @return void
// */
//void pwm_service_task(
//        unsigned motor_id,
//        PwmPorts &ports,
//        server interface UpdatePWM i_update_pwm,
//        server interface UpdateBrake i_update_brake,
//        int ifm_tile_usec
//)
//{
//    int duty_start_brake    = 3000;
//    int duty_maintain_brake = 3000;
//    int brake_start         = 0;
//    unsigned char  brake_defined_II    = 0b1111;
//
//    unsigned int half_sync_inc=0;
//    unsigned int pwm_max_value=0;
//    unsigned int pwm_deadtime =0;
//
//    PWM_ARRAY_TYP pwm_ctrl_s ; // Structure containing double-buffered PWM output data
//    PWM_SERV_TYP  pwm_serv_s ; // Structure containing PWM server control data
//    PWM_COMMS_TYP pwm_comms_s; // Structure containing PWM communication data
//
//    PWM_ARRAY_TYP pwm_ctrl_s_start_brake ; // Structure containing double-buffered PWM output data
//    PWM_COMMS_TYP pwm_comms_s_start_brake; // Structure containing PWM communication data
//
//    //parameters for maintaining the brake
//    PWM_ARRAY_TYP pwm_ctrl_s_maintain_brake ; // Structure containing double-buffered PWM output data
//    PWM_COMMS_TYP pwm_comms_s_maintain_brake; // Structure containing PWM communication data
//
//    timer t;
//    unsigned ts;
//
//    if(ifm_tile_usec==250)
//    {
//        half_sync_inc = 8192;
//        pwm_max_value=16384;
//        pwm_deadtime=1500;
//
//        //Set freq to 250MHz (always needed for proper timing)
//        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
//    }
//    else if(ifm_tile_usec==100)
//    {
//        half_sync_inc = 4096;
//        pwm_max_value=8192;
//        pwm_deadtime=600;
//    }
//    else if (ifm_tile_usec!=100 && ifm_tile_usec!=250)
//    {
//        while(1);//error state!!!
//    }
//
//
//    t :> ts;
//    t when timerafter (ts + (4000*20*250)) :> void;    //proper task startup
//
////    select
////    {
////    case i_update_brake.update_brake_control_data(int _duty_start_brake, int _duty_maintain_brake, int _period_start_brake):
////            duty_start_brake    = _duty_start_brake;
////            duty_maintain_brake = _duty_maintain_brake;
////            brake_start         = _period_start_brake;
////            brake_defined_II    = 0b1111;
////
////
////            //parameters for starting the brake
////            pwm_comms_s_start_brake.params.widths[0] =  duty_start_brake;
////            pwm_comms_s_start_brake.params.widths[1] =  duty_start_brake;
////            pwm_comms_s_start_brake.params.widths[2] =  duty_start_brake;
////
////            pwm_comms_s_start_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
////            pwm_comms_s_start_brake.buf = 0;
////
////            convert_all_pulse_widths( pwm_comms_s_start_brake ,pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
////
////            pwm_comms_s_maintain_brake.params.widths[0] = duty_maintain_brake;
////            pwm_comms_s_maintain_brake.params.widths[1] = duty_maintain_brake;
////            pwm_comms_s_maintain_brake.params.widths[2] = duty_maintain_brake;
////
////            pwm_comms_s_maintain_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
////            pwm_comms_s_maintain_brake.buf = 0;
////
////            convert_all_pulse_widths( pwm_comms_s_maintain_brake ,pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
////
////            break;
////
////    default:
////        break;
////    }
//
//    //parameters for starting the brake
//    pwm_comms_s_start_brake.params.widths[0] =  duty_start_brake;
//    pwm_comms_s_start_brake.params.widths[1] =  duty_start_brake;
//    pwm_comms_s_start_brake.params.widths[2] =  duty_start_brake;
//
//    pwm_comms_s_start_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
//    pwm_comms_s_start_brake.buf = 0;
//
//    convert_all_pulse_widths( pwm_comms_s_start_brake ,pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//    pwm_comms_s_maintain_brake.params.widths[0] = duty_maintain_brake;
//    pwm_comms_s_maintain_brake.params.widths[1] = duty_maintain_brake;
//    pwm_comms_s_maintain_brake.params.widths[2] = duty_maintain_brake;
//
//    pwm_comms_s_maintain_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
//    pwm_comms_s_maintain_brake.buf = 0;
//
//    convert_all_pulse_widths( pwm_comms_s_maintain_brake ,pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//    unsigned pattern=0; // Bit-pattern on port
//    int pwm_on  =0;
//    int brake_active  = 0;
//    int brake_counter = 0;
//
//    unsigned char brake_defined = 0b0000;
//    brake_defined = ((!isnull(ports.p_pwm_phase_d))&brake_defined_II);
//
//    // initialize PWM
//    pwm_serv_s.id = motor_id;  // Assign motor identifier
//    pwm_comms_s.params.id = 0; // Unique Motor identifier e.g. 0 or 1
//    pwm_comms_s.buf = 0;
//
//    pwm_comms_s.params.widths[0] = 4000;
//    pwm_comms_s.params.widths[1] = 4000;
//    pwm_comms_s.params.widths[2] = 4000;
//
//    convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//    // Find out value of time clock on an output port, WITHOUT changing port value
//    pattern = peek( ports.p_pwm[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
//    pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[0] ,1 ,pattern ); // Re-load output port with same bit-value
//
//    pwm_serv_s.data_ready = 1; // Signal new data ready. NB this happened in init_pwm_data()
//
//    while (1)
//    {
//        select
//        {
//        case i_update_brake.update_brake_control_data(int _duty_start_brake, int _duty_maintain_brake, int _period_start_brake):
//                duty_start_brake    = _duty_start_brake;
//                duty_maintain_brake = _duty_maintain_brake;
//                brake_start         = _period_start_brake;
//                brake_defined_II    = 0b1111;
//
//                //parameters for starting the brake
//                pwm_comms_s_start_brake.params.widths[0] =  duty_start_brake;
//                pwm_comms_s_start_brake.params.widths[1] =  duty_start_brake;
//                pwm_comms_s_start_brake.params.widths[2] =  duty_start_brake;
//
//                pwm_comms_s_start_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
//                pwm_comms_s_start_brake.buf = 0;
//
//                convert_all_pulse_widths( pwm_comms_s_start_brake ,pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//                pwm_comms_s_maintain_brake.params.widths[0] = duty_maintain_brake;
//                pwm_comms_s_maintain_brake.params.widths[1] = duty_maintain_brake;
//                pwm_comms_s_maintain_brake.params.widths[2] = duty_maintain_brake;
//
//                pwm_comms_s_maintain_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
//                pwm_comms_s_maintain_brake.buf = 0;
//
//                convert_all_pulse_widths( pwm_comms_s_maintain_brake ,pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//                break;
//        case i_update_pwm.status() -> {int status}:
//                status = ACTIVE;
//                break;
//
//        case i_update_pwm.update_server_control_data(int pwm_a, int pwm_b, int pwm_c, int received_pwm_on, int received_brake_active, int recieved_safe_torque_off_mode):
//                pwm_comms_s.params.widths[0] =  pwm_a;
//                pwm_comms_s.params.widths[1] =  pwm_b;
//                pwm_comms_s.params.widths[2] =  pwm_c;
//                convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
//
//                if(recieved_safe_torque_off_mode ==0)
//                    pwm_on     = received_pwm_on;
//                else if(recieved_safe_torque_off_mode ==1)
//                    pwm_on     = 0;
//
//                if(received_brake_active==0)  brake_active = 0;
//
//                if((brake_active == 0)&&(received_brake_active==1))
//                {
//                    brake_counter=0;
//                    brake_active = 1;
//                }
//
//                pattern = peek( ports.p_pwm[_PWM_PHASE_A] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
//                pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[_PWM_PHASE_A] ,1 ,pattern ); // Re-load output port with same bit-value
//                pwm_serv_s.ref_time += half_sync_inc;
//
//                break;
//
//        case i_update_pwm.safe_torque_off_enabled():
//
//            pwm_on     = 0;
//
//            pattern = peek( ports.p_pwm[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
//            pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[0] ,1 ,pattern ); // Re-load output port with same bit-value
//            pwm_serv_s.ref_time += half_sync_inc;
//
//            // Rising edges - these have negative time offsets - 44 Cycles
//            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: 0xFFFFFFFF;
//            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: 0xFFFFFFFF;
//            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;
//
//            // Falling edges - these have positive time offsets - 44 Cycles
//            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: 0xFFFFFFFF;
//            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: 0xFFFFFFFF;
//            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
//            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;
//
//            break;
//        }
//
//        if (pwm_on)
//        {
//            // Rising edges - these have negative time offsets - 44 Cycles
//            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.pattern;
//
//            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.pattern;
//
//            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//        }
//
//        if(brake_active && brake_defined)
//        {
//            if(brake_counter < brake_start)
//            {
//                brake_counter++;
//                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//            }
//            else
//            {
//                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//            }
//        }
//
//        if (pwm_on)
//        {
//            // Falling edges - these have positive time offsets - 44 Cycles
//            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.pattern;
//
//            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.pattern;
//
//            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//        }
//
//        if(brake_active && brake_defined)
//        {
//            if(brake_counter < brake_start)
//            {
//                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//            }
//            else
//            {
//                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
//                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
//            }
//        }
//
//    } // while(1)
//
//} // pwm_service_task

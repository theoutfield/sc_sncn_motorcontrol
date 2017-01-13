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
#include "app_global.h"
#include "pwm_convert_width.h"

#include <motor_control_interfaces.h>

#include <a4935.h>
#include <mc_internal_constants.h>


/*****************************************************************************/
static void do_pwm_port_config(PwmPorts &ports)
{
    unsigned i;

    // Loop through PWM phases
    for (i = 0; i < _NUM_PWM_PHASES; i++)
    {   // Configure ports for this phase

        configure_out_port( ports.p_pwm[i] , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv[i] , ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv[i] );
    }

    if (!isnull(ports.p_pwm_phase_d))
    {
        configure_out_port( ports.p_pwm_phase_d , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_phase_d_inv , ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_phase_d_inv );
    }

    // Check of ADC synchronisation is being used
    if (1 == _LOCK_ADC_TO_PWM)
    {   // ADC synchronisation activated

        // Configure dummy input port used to send ADC synchronisation pulse
        configure_in_port( ports.dummy_port, ports.clk );
    } // if (1 == LOCK_ADC_TO_PWM)


} // do_pwm_port_config

/*****************************************************************************/
static void do_pwm_port_config_general(PwmPortsGeneral &ports)
{

    if (!isnull(ports.p_pwm_a))
    {
        configure_out_port( ports.p_pwm_a    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_a, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_a);
    }

    if (!isnull(ports.p_pwm_b))
    {
        configure_out_port( ports.p_pwm_b    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_b, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_b);
    }

    if (!isnull(ports.p_pwm_c))
    {
        configure_out_port( ports.p_pwm_c    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_c, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_c);
    }

    if (!isnull(ports.p_pwm_u))
    {
        configure_out_port( ports.p_pwm_u    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_u, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_u);
    }

    if (!isnull(ports.p_pwm_v))
    {
        configure_out_port( ports.p_pwm_v    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_v, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_v);
    }

    if (!isnull(ports.p_pwm_w))
    {
        configure_out_port( ports.p_pwm_w    , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
        configure_out_port( ports.p_pwm_inv_w, ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
        set_port_inv( ports.p_pwm_inv_w);
    }

    // Check of ADC synchronisation is being used
    if (1==_LOCK_ADC_TO_PWM && (!isnull(ports.dummy_port)))
    {
        // Configure dummy input port used to send ADC synchronisation pulse
        configure_in_port( ports.dummy_port, ports.clk );
    }

} // do_pwm_port_config


/*****************************************************************************/
void pwm_config(PwmPorts &ports)
{
    //    int motor_cnt; // motor counter

    // Configure clock rate to PLATFORM_REFERENCE_MHZ/1 (100 MHz) -> in our application it is 250 MHz
    //configure_clock_rate( ports.clk ,PLATFORM_REFERENCE_MHZ ,1 );

    do_pwm_port_config(ports);

    start_clock( ports.clk ); // Start common PWM clock, once all ports configured

    ports.p_pwm[_PWM_PHASE_A]  <: 0x00000000;
    ports.p_pwm_inv[_PWM_PHASE_A]  <: 0xFFFFFFFF;

    ports.p_pwm[_PWM_PHASE_B]  <: 0x00000000;
    ports.p_pwm_inv[_PWM_PHASE_B]  <: 0xFFFFFFFF;

    ports.p_pwm[_PWM_PHASE_C]  <: 0x00000000;
    ports.p_pwm_inv[_PWM_PHASE_C]  <: 0xFFFFFFFF;

    if (!isnull(ports.p_pwm_phase_d))
    {
        ports.p_pwm_phase_d  <: 0x00000000;
        ports.p_pwm_phase_d_inv  <: 0xFFFFFFFF;
    }


} // foc_pwm_config

/*****************************************************************************/
void pwm_config_general(PwmPortsGeneral &ports)
{
    // Configure clock rate to PLATFORM_REFERENCE_MHZ/1 (100 MHz) -> in our application it is 250 MHz
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

} // foc_pwm_config


/*****************************************************************************/
void predriver(FetDriverPorts &fet_driver_ports)
{
    const unsigned t_delay = 300*100;//300 us
    unsigned int ts;
    int check_fet;
    int init_state = INIT_BUSY;
    timer t;

    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
    {
        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
        t when timerafter (ts + t_delay) :> ts;
    }

    if(!isnull(fet_driver_ports.p_coast))
    {
        fet_driver_ports.p_coast :> check_fet;
        init_state = check_fet;
    }
    else
    {
        init_state = 1;
    }


} // foc_pwm_config


void pwm_check_general(PwmPortsGeneral &ports)
{

    while(1)
    {
        if (!isnull(ports.p_pwm_a))
        {
            ports.p_pwm_a     <: 0xFFFFFFFF;
            ports.p_pwm_inv_a <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_b))
        {
            ports.p_pwm_b     <: 0xFFFFFFFF;
            ports.p_pwm_inv_b <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_c))
        {

            ports.p_pwm_c     <: 0xFFFFFFFF;
            ports.p_pwm_inv_c <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_u))
        {
            ports.p_pwm_u     <: 0xFFFFFFFF;
            ports.p_pwm_inv_u <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_v))
        {
            ports.p_pwm_v     <: 0xFFFFFFFF;
            ports.p_pwm_inv_v <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_w))
        {
            ports.p_pwm_w     <: 0xFFFFFFFF;
            ports.p_pwm_inv_w <: 0xFFFFFFFF;
        }
        delay_microseconds(100);


        if (!isnull(ports.p_pwm_a))
        {
            ports.p_pwm_a     <: 0x00000000;
            ports.p_pwm_inv_a <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_b))
        {
            ports.p_pwm_b     <: 0x00000000;
            ports.p_pwm_inv_b <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_c))
        {
            ports.p_pwm_c     <: 0x00000000;
            ports.p_pwm_inv_c <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_u))
        {
            ports.p_pwm_u     <: 0x00000000;
            ports.p_pwm_inv_u <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_v))
        {
            ports.p_pwm_v     <: 0x00000000;
            ports.p_pwm_inv_v <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_w))
        {
            ports.p_pwm_w     <: 0x00000000;
            ports.p_pwm_inv_w <: 0xFFFFFFFF;
        }
        delay_microseconds(3);


        if (!isnull(ports.p_pwm_a))
        {
            ports.p_pwm_a     <: 0x00000000;
            ports.p_pwm_inv_a <: 0x00000000;
        }

        if (!isnull(ports.p_pwm_b))
        {
            ports.p_pwm_b     <: 0x00000000;
            ports.p_pwm_inv_b <: 0x00000000;
        }

        if (!isnull(ports.p_pwm_c))
        {
            ports.p_pwm_c     <: 0x00000000;
            ports.p_pwm_inv_c <: 0x00000000;
        }

        if (!isnull(ports.p_pwm_u))
        {
            ports.p_pwm_u     <: 0x00000000;
            ports.p_pwm_inv_u <: 0x00000000;
        }

        if (!isnull(ports.p_pwm_v))
        {
            ports.p_pwm_v     <: 0x00000000;
            ports.p_pwm_inv_v <: 0x00000000;
        }

        if (!isnull(ports.p_pwm_w))
        {
            ports.p_pwm_w     <: 0x00000000;
            ports.p_pwm_inv_w <: 0x00000000;
        }
        delay_microseconds(100);

        if (!isnull(ports.p_pwm_a))
        {
            ports.p_pwm_a     <: 0x00000000;
            ports.p_pwm_inv_a <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_b))
        {
            ports.p_pwm_b     <: 0x00000000;
            ports.p_pwm_inv_b <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_c))
        {
            ports.p_pwm_c     <: 0x00000000;
            ports.p_pwm_inv_c <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_u))
        {
            ports.p_pwm_u     <: 0x00000000;
            ports.p_pwm_inv_u <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_v))
        {
            ports.p_pwm_v     <: 0x00000000;
            ports.p_pwm_inv_v <: 0xFFFFFFFF;
        }

        if (!isnull(ports.p_pwm_w))
        {
            ports.p_pwm_w     <: 0x00000000;
            ports.p_pwm_inv_w <: 0xFFFFFFFF;
        }
        delay_microseconds(3);

    }

}


void pwm_checkl(PwmPorts &ports)
{

    while(1)
    {

        ports.p_pwm[_PWM_PHASE_A]  <: 0xFFFFFFFF;
        ports.p_pwm_inv[_PWM_PHASE_A]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_B]  <: 0xFFFFFFFF;
        ports.p_pwm_inv[_PWM_PHASE_B]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_C]  <: 0xFFFFFFFF;
        ports.p_pwm_inv[_PWM_PHASE_C]  <: 0xFFFFFFFF;
        delay_microseconds(100);


        ports.p_pwm[_PWM_PHASE_A]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_A]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_B]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_B]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_C]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_C]  <: 0xFFFFFFFF;
        delay_microseconds(3);


        ports.p_pwm[_PWM_PHASE_A]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_A]  <: 0x00000000;

        ports.p_pwm[_PWM_PHASE_B]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_B]  <: 0x00000000;

        ports.p_pwm[_PWM_PHASE_C]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_C]  <: 0x00000000;
        delay_microseconds(100);


        ports.p_pwm[_PWM_PHASE_A]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_A]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_B]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_B]  <: 0xFFFFFFFF;

        ports.p_pwm[_PWM_PHASE_C]  <: 0x00000000;
        ports.p_pwm_inv[_PWM_PHASE_C]  <: 0xFFFFFFFF;
        delay_microseconds(3);

    }

}
void pwm_service_task( // Implementation of the Centre-aligned, High-Low pair, PWM server, with ADC sync
        unsigned motor_id, // Motor identifier
        PwmPorts &ports,
        server interface update_pwm i_update_pwm,
        int duty_start_brake,
        int duty_maintain_brake,
        int time_start_brake,
        int ifm_tile_usec
)
{
    unsigned int half_sync_inc=0;
    unsigned int pwm_max_value=0;
    unsigned int pwm_deadtime =0;


    timer t;
    unsigned ts;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (4000*20*250)) :> void;


    if(ifm_tile_usec==250)
    {
        half_sync_inc = 8192;
        pwm_max_value=16384;
        pwm_deadtime=1500;

        //Set freq to 250MHz (always needed for proper timing)
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }
    else if(ifm_tile_usec==100)
    {
        half_sync_inc = 4096;
        pwm_max_value=8192;
        pwm_deadtime=600;
    }
    else if (ifm_tile_usec!=100 && ifm_tile_usec!=250)
    {
        while(1);//error state!!!
    }


    PWM_ARRAY_TYP pwm_ctrl_s ; // Structure containing double-buffered PWM output data
    PWM_SERV_TYP  pwm_serv_s ; // Structure containing PWM server control data
    PWM_COMMS_TYP pwm_comms_s; // Structure containing PWM communication data


    PWM_ARRAY_TYP pwm_ctrl_s_start_brake ; // Structure containing double-buffered PWM output data
    PWM_COMMS_TYP pwm_comms_s_start_brake; // Structure containing PWM communication data



    //parameters for starting the brake
    pwm_comms_s_start_brake.params.widths[0] = duty_start_brake;
    pwm_comms_s_start_brake.params.widths[1] =  duty_start_brake;
    pwm_comms_s_start_brake.params.widths[2] =  duty_start_brake;

    pwm_comms_s_start_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s_start_brake.buf = 0;

    convert_all_pulse_widths( pwm_comms_s_start_brake ,pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles


    //parameters for maintaining the brake

    PWM_ARRAY_TYP pwm_ctrl_s_maintain_brake ; // Structure containing double-buffered PWM output data
    PWM_COMMS_TYP pwm_comms_s_maintain_brake; // Structure containing PWM communication data

    pwm_comms_s_maintain_brake.params.widths[0] = duty_maintain_brake;
    pwm_comms_s_maintain_brake.params.widths[1] = duty_maintain_brake;
    pwm_comms_s_maintain_brake.params.widths[2] = duty_maintain_brake;

    pwm_comms_s_maintain_brake.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s_maintain_brake.buf = 0;

    convert_all_pulse_widths( pwm_comms_s_maintain_brake ,pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles



    unsigned pattern=0; // Bit-pattern on port

    //    int pwm_test=0;
    int pwm_on  =0;

    int brake_active  = 0;
    int brake_counter = 0;
    int brake_start   = (time_start_brake*15000)/1000;

    //    int pwm_flag=0;

    unsigned char brake_defined = 0b0000;
    brake_defined = !isnull(ports.p_pwm_phase_d);

    // initialize PWM
    pwm_serv_s.id = motor_id; // Assign motor identifier
    pwm_comms_s.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s.buf = 0;

    pwm_comms_s.params.widths[0] = 4000;
    pwm_comms_s.params.widths[1] = 4000;
    pwm_comms_s.params.widths[2] = 4000;

    convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles

    // Find out value of time clock on an output port, WITHOUT changing port value
    pattern = peek( ports.p_pwm[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
    pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[0] ,1 ,pattern ); // Re-load output port with same bit-value

    pwm_serv_s.data_ready = 1; // Signal new data ready. NB this happened in init_pwm_data()



    /* This loop requires at least ~280 cycles, which means the PWM period must be at least 512 cycles.
     * If convert_all_pulse_widths was optimised for speed, maybe a PWM period of 256 cycles would be possible
     */
    while (1)
    {
        select
        {
        case i_update_pwm.status() -> {int status}:
                status = ACTIVE;
                break;

        case i_update_pwm.update_server_control_data(int pwm_a, int pwm_b, int pwm_c, int received_pwm_on, int received_brake_active, int recieved_safe_torque_off_mode):
                pwm_comms_s.params.widths[0] =  pwm_a;
                pwm_comms_s.params.widths[1] =  pwm_b;
                pwm_comms_s.params.widths[2] =  pwm_c;
                convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles

                if(recieved_safe_torque_off_mode ==0)
                    pwm_on     = received_pwm_on;
                else if(recieved_safe_torque_off_mode ==1)
                    pwm_on     = 0;

                if(received_brake_active==0)  brake_active = 0;

                if((brake_active == 0)&&(received_brake_active==1))
                {
                    brake_counter=0;
                    brake_active = 1;
                }



                pattern = peek( ports.p_pwm[_PWM_PHASE_A] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
                pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[_PWM_PHASE_A] ,1 ,pattern ); // Re-load output port with same bit-value
                pwm_serv_s.ref_time += half_sync_inc;

                break;

        case i_update_pwm.safe_torque_off_enabled():

            pwm_on     = 0;

            pattern = peek( ports.p_pwm[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
            pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[0] ,1 ,pattern ); // Re-load output port with same bit-value
            pwm_serv_s.ref_time += half_sync_inc;


            // Rising edges - these have negative time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: 0xFFFFFFFF;
            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: 0xFFFFFFFF;
            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;

            // Falling edges - these have positive time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: 0xFFFFFFFF;
            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: 0xFFFFFFFF;
            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;



            break;
        }

        if (pwm_on)
        {
            // Rising edges - these have negative time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.pattern;

            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.pattern;

            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
        }

        if(brake_active && brake_defined)
        {
            if(brake_counter < brake_start)
            {
                brake_counter++;

                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
            }
            else
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
            }
        }

        if (pwm_on)
        {
            // Falling edges - these have positive time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.pattern;

            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.pattern;

            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
        }

        if(brake_active && brake_defined)
        {
            if(brake_counter < brake_start)
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_brake.buf_data[pwm_comms_s_start_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
            }
            else
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_brake.buf_data[pwm_comms_s_maintain_brake.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
            }
        }

    } // while(1)


} // pwm_service_task
/*****************************************************************************/

/*
 *
 * */
void pwm_service_general(
        PwmPortsGeneral &ports,
        server interface update_pwm_general i_update_pwm,
        int duty_start_brake,
        int duty_maintain_brake,
        int time_start_brake,
        int ifm_tile_usec,
        int commutation_frq
)
{
    unsigned int half_sync_inc=0;
    unsigned int pwm_max_value=0;
    unsigned int pwm_deadtime =0;

    timer t;
    unsigned ts=0, time=0, t_start=0, t_end=0;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (4000*20*250)) :> void;


    if(ifm_tile_usec==250)
    {
        if(commutation_frq==15)
        {
            half_sync_inc = 8192;
            pwm_max_value=16384;
            pwm_deadtime=1500;
        }
        //Set freq to 250MHz (always needed for proper timing)
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }
    else if(ifm_tile_usec==100)
    {
        if(commutation_frq==12)
        {
            half_sync_inc = 4096;
            pwm_max_value=8192;
            pwm_deadtime=600;
        }

        if(commutation_frq==24)
        {
            half_sync_inc = 2048;
            pwm_max_value=4096;
            pwm_deadtime=600;
        }

    }
    else if (ifm_tile_usec!=100 && ifm_tile_usec!=250)
    {
        while(1);//error state!!!
    }

    PWM_ARRAY_TYP pwm_ctrl_s1, pwm_ctrl_s2 ;  // Structure containing double-buffered PWM output data
    PWM_SERV_TYP  pwm_serv_s1, pwm_serv_s2 ;  // Structure containing PWM server control data
    PWM_COMMS_TYP pwm_comms_s1, pwm_comms_s2; // Structure containing PWM communication data

    unsigned pattern=0; // Bit-pattern on port

    int pwm_on  =0;

    int brake_active  = 0;
    int brake_counter = 0;
    int brake_start   = (time_start_brake*15000)/1000;

    unsigned char phase_a_defined=0b0000, phase_b_defined=0b0000, phase_c_defined=0b0000,
                  phase_u_defined=0b0000, phase_v_defined=0b0000, phase_w_defined=0b0000;

    unsigned char phase_a_inv_defined=0b0000, phase_b_inv_defined=0b0000, phase_c_inv_defined=0b0000,
                  phase_u_inv_defined=0b0000, phase_v_inv_defined=0b0000, phase_w_inv_defined=0b0000;

    phase_a_defined = !isnull(ports.p_pwm_a);
    phase_b_defined = !isnull(ports.p_pwm_b);
    phase_c_defined = !isnull(ports.p_pwm_c);
    phase_u_defined = !isnull(ports.p_pwm_u);
    phase_v_defined = !isnull(ports.p_pwm_v);
    phase_w_defined = !isnull(ports.p_pwm_w);

    phase_a_inv_defined = !isnull(ports.p_pwm_inv_a);
    phase_b_inv_defined = !isnull(ports.p_pwm_inv_b);
    phase_c_inv_defined = !isnull(ports.p_pwm_inv_c);
    phase_u_inv_defined = !isnull(ports.p_pwm_inv_u);
    phase_v_inv_defined = !isnull(ports.p_pwm_inv_v);
    phase_w_inv_defined = !isnull(ports.p_pwm_inv_w);

    // initialize PWM
    pwm_serv_s1.id = 0; // Assign motor identifier
    pwm_serv_s2.id = 0; // Assign motor identifier
    pwm_comms_s1.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s2.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s1.buf = 0;
    pwm_comms_s2.buf = 0;

    pwm_comms_s1.params.widths[0] = 2000;
    pwm_comms_s1.params.widths[1] = 2000;
    pwm_comms_s1.params.widths[2] = 2000;
    pwm_comms_s2.params.widths[0] = 2000;
    pwm_comms_s2.params.widths[1] = 2000;
    pwm_comms_s2.params.widths[2] = 2000;

    convert_all_pulse_widths( pwm_comms_s1 ,pwm_ctrl_s1.buf_data[pwm_comms_s1.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
    convert_all_pulse_widths( pwm_comms_s2 ,pwm_ctrl_s2.buf_data[pwm_comms_s2.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles


    pwm_serv_s1.data_ready = 1; // Signal new data ready. NB this happened in init_pwm_data()
    pwm_serv_s2.data_ready = 1; // Signal new data ready. NB this happened in init_pwm_data()

    // Find out value of time clock on an output port, WITHOUT changing port value
    pattern = peek( ports.p_pwm_a ); // Find out value on 1-bit port. NB Only LS-bit is relevant
    pwm_serv_s1.ref_time = partout_timestamped( ports.p_pwm_a ,1 ,pattern ); // Re-load output port with same bit-value
    pwm_serv_s2.ref_time = pwm_serv_s1.ref_time;
    t :> time;

    /* This loop requires at least ~280 cycles, which means the PWM period must be at least 512 cycles.
     * If convert_all_pulse_widths was optimised for speed, maybe a PWM period of 256 cycles would be possible
     */
    while (1)
    {
        select
        {
        case i_update_pwm.status() -> {int status}:
                status = ACTIVE;
                break;

        case i_update_pwm.update_server_control_data(
                int pwm_a, int pwm_b, int pwm_c,
                int pwm_u, int pwm_v, int pwm_w,
                int received_pwm_on, int received_brake_active, int recieved_safe_torque_off_mode) -> unsigned int t_calculation:
                        t :> t_start;
                        pwm_comms_s1.params.widths[0] =  pwm_a;
                        pwm_comms_s1.params.widths[1] =  pwm_b;
                        pwm_comms_s1.params.widths[2] =  pwm_c;

                        pwm_comms_s2.params.widths[0] =  pwm_u;
                        pwm_comms_s2.params.widths[1] =  pwm_v;
                        pwm_comms_s2.params.widths[2] =  pwm_w;

                convert_all_pulse_widths( pwm_comms_s1 ,pwm_ctrl_s1.buf_data[pwm_comms_s1.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles
                convert_all_pulse_widths( pwm_comms_s2 ,pwm_ctrl_s2.buf_data[pwm_comms_s2.buf], pwm_max_value, pwm_deadtime); // Max 178 Cycles

                if(recieved_safe_torque_off_mode ==0)
                    pwm_on     = received_pwm_on;
                else if(recieved_safe_torque_off_mode ==1)
                    pwm_on     = 0;

                if(received_brake_active==0)  brake_active = 0;

                if((brake_active == 0)&&(received_brake_active==1))
                {
                    brake_counter=0;
                    brake_active = 1;
                }
                t :> t_end;

                t_calculation = t_end - t_start;
                break;

        case i_update_pwm.safe_torque_off_enabled():

            break;

        case t when timerafter(time) :> void:

                pwm_serv_s1.ref_time += half_sync_inc;
                pwm_serv_s2.ref_time += half_sync_inc;

                if(phase_a_defined) ports.p_pwm_a           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[0].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[0].hi.pattern;
                if(phase_a_inv_defined) ports.p_pwm_inv_a   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[0].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[0].lo.pattern;
                if(phase_b_defined) ports.p_pwm_b           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[1].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[1].hi.pattern;
                if(phase_b_inv_defined) ports.p_pwm_inv_b   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[1].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[1].lo.pattern;
                if(phase_c_defined) ports.p_pwm_c           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[2].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[2].hi.pattern;
                if(phase_c_inv_defined) ports.p_pwm_inv_c   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[2].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].rise_edg.phase_data[2].lo.pattern;
                if(phase_u_defined) ports.p_pwm_u           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[0].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[0].hi.pattern;
                if(phase_u_inv_defined) ports.p_pwm_inv_u   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[0].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[0].lo.pattern;
                if(phase_v_defined) ports.p_pwm_v           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[1].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[1].hi.pattern;
                if(phase_v_inv_defined) ports.p_pwm_inv_v   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[1].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[1].lo.pattern;
                if(phase_w_defined) ports.p_pwm_w           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[2].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[2].hi.pattern;
                if(phase_w_inv_defined) ports.p_pwm_inv_w   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[2].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].rise_edg.phase_data[2].lo.pattern;


                if(phase_a_defined) ports.p_pwm_a           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[0].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[0].hi.pattern;
                if(phase_a_inv_defined) ports.p_pwm_inv_a   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[0].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[0].lo.pattern;
                if(phase_b_defined) ports.p_pwm_b           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[1].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[1].hi.pattern;
                if(phase_b_inv_defined) ports.p_pwm_inv_b   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[1].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[1].lo.pattern;
                if(phase_c_defined) ports.p_pwm_c           @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[2].hi.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[2].hi.pattern;
                if(phase_c_inv_defined) ports.p_pwm_inv_c   @ (PORT_TIME_TYP)(pwm_serv_s1.ref_time + pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[2].lo.time_off) <: pwm_ctrl_s1.buf_data[pwm_comms_s1.buf].fall_edg.phase_data[2].lo.pattern;
                if(phase_u_defined) ports.p_pwm_u           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[0].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[0].hi.pattern;
                if(phase_u_inv_defined) ports.p_pwm_inv_u   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[0].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[0].lo.pattern;
                if(phase_v_defined) ports.p_pwm_v           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[1].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[1].hi.pattern;
                if(phase_v_inv_defined) ports.p_pwm_inv_v   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[1].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[1].lo.pattern;
                if(phase_w_defined) ports.p_pwm_w           @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[2].hi.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[2].hi.pattern;
                if(phase_w_inv_defined) ports.p_pwm_inv_w   @ (PORT_TIME_TYP)(pwm_serv_s2.ref_time + pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[2].lo.time_off) <: pwm_ctrl_s2.buf_data[pwm_comms_s2.buf].fall_edg.phase_data[2].lo.pattern;

                pwm_serv_s1.ref_time += half_sync_inc;
                pwm_serv_s2.ref_time += half_sync_inc;
                time += pwm_max_value;
            break;
        }

    } // while(1)


} // pwm_service_task
/*****************************************************************************/

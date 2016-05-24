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
#include <lookup_tables.h>


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

    configure_out_port( ports.p_pwm_phase_d , ports.clk ,0 );     // Set initial value of port to 0 (Switched Off)
    configure_out_port( ports.p_pwm_phase_d_inv , ports.clk ,0 ); // Set initial value of port to 0 (Switched Off)
    set_port_inv( ports.p_pwm_phase_d_inv );


    // Check of ADC synchronisation is being used
    if (1 == _LOCK_ADC_TO_PWM)
    {   // ADC synchronisation activated

        // Configure dummy input port used to send ADC synchronisation pulse
        configure_in_port( ports.dummy_port, ports.clk );
    } // if (1 == LOCK_ADC_TO_PWM)


} // do_pwm_port_config
/*****************************************************************************/
void pwm_config(PwmPorts &ports)
{
    int motor_cnt; // motor counter

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

    ports.p_pwm_phase_d  <: 0x00000000;
    ports.p_pwm_phase_d_inv  <: 0xFFFFFFFF;




} // foc_pwm_config

void update_pwm(control_variables& cv, PWM_COMMS_TYP& pwm_comms_s)
{
    pwm_comms_s.params.widths[0] =  cv.pwm_values[0];
    pwm_comms_s.params.widths[1] =  cv.pwm_values[1];
    pwm_comms_s.params.widths[2] =  cv.pwm_values[2];
}

void pwm_check(PwmPorts &ports)
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
        server interface update_pwm i_update_pwm
)
{

    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    PWM_ARRAY_TYP pwm_ctrl_s ; // Structure containing double-buffered PWM output data
    PWM_SERV_TYP  pwm_serv_s ; // Structure containing PWM server control data
    PWM_COMMS_TYP pwm_comms_s; // Structure containing PWM communication data


    PWM_ARRAY_TYP pwm_ctrl_s_start_break ; // Structure containing double-buffered PWM output data
    PWM_COMMS_TYP pwm_comms_s_start_break; // Structure containing PWM communication data

    int break_active  = 0;
    int break_counter =0;

    //parameters for starting the break
    pwm_comms_s_start_break.params.widths[0] = 8500;
    pwm_comms_s_start_break.params.widths[1] = 8500;
    pwm_comms_s_start_break.params.widths[2] = 8500;

    pwm_comms_s_start_break.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s_start_break.buf = 0;

    convert_all_pulse_widths( pwm_comms_s_start_break ,pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf] ); // Max 178 Cycles


    //parameters for maintaining the break

    PWM_ARRAY_TYP pwm_ctrl_s_maintain_break ; // Structure containing double-buffered PWM output data
    PWM_COMMS_TYP pwm_comms_s_maintain_break; // Structure containing PWM communication data

    pwm_comms_s_maintain_break.params.widths[0] = 1000;
    pwm_comms_s_maintain_break.params.widths[1] = 1000;
    pwm_comms_s_maintain_break.params.widths[2] = 1000;

    pwm_comms_s_maintain_break.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s_maintain_break.buf = 0;

    convert_all_pulse_widths( pwm_comms_s_maintain_break ,pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf] ); // Max 178 Cycles



    unsigned pattern=0; // Bit-pattern on port

    int pwm_test=0;
    int pwm_on  =0;

    int pwm_flag=0;

    // initialize PWM
    pwm_serv_s.id = motor_id; // Assign motor identifier
    pwm_comms_s.params.id = 0; // Unique Motor identifier e.g. 0 or 1
    pwm_comms_s.buf = 0;

    pwm_comms_s.params.widths[0] = 1500;
    pwm_comms_s.params.widths[1] = 1500;
    pwm_comms_s.params.widths[2] = 1500;

    convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf] ); // Max 178 Cycles

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
        case i_update_pwm.update_server_control_data(PWM_ARRAY_TYP recieved_pwm_ctrl_s, int recieved_pwm_test, int recieved_pwm_on, int recieved_break_active):
                pwm_ctrl_s = recieved_pwm_ctrl_s;

                pwm_on     = recieved_pwm_on;
                if(pwm_on==0) break_counter=0;

                break_active = recieved_break_active;

                pattern = peek( ports.p_pwm[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
                pwm_serv_s.ref_time = partout_timestamped( ports.p_pwm[0] ,1 ,pattern ); // Re-load output port with same bit-value
                pwm_serv_s.ref_time += _HALF_SYNC_INCREMENT;

                break;
        }

        if (pwm_on==1)
        {

            // Rising edges - these have negative time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_A].lo.pattern;

            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_B].lo.pattern;

            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;

            if(break_counter < 15000)
            {
                break_counter++;
                if(break_active == 1)
                {
                    ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                    ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;
                }
            }
            else if(break_active == 1)
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.pattern;

            }
            else if(break_active == 0)
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;
            }


            // Falling edges - these have positive time offsets - 44 Cycles
            ports.p_pwm[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_A].lo.pattern;

            ports.p_pwm[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_B].lo.pattern;

            ports.p_pwm[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
            ports.p_pwm_inv[_PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;

            if(break_counter < 15000)
            {
                if(break_active == 1)
                {
                    ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                    ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_start_break.buf_data[pwm_comms_s_start_break.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;
                }
            }
            else if(break_active == 1)
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].fall_edg.phase_data[_PWM_PHASE_C].hi.pattern;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s_maintain_break.buf_data[pwm_comms_s_maintain_break.buf].fall_edg.phase_data[_PWM_PHASE_C].lo.pattern;

            }
            else if(break_active == 0)
            {
                ports.p_pwm_phase_d @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].hi.time_off) <: 0x00000000;
                ports.p_pwm_phase_d_inv @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[_PWM_PHASE_C].lo.time_off) <: 0xFFFFFFFF;
            }


        }

        else if (pwm_on==0)
        {
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
        }

    } // while(1)


} // pwm_service_task
/*****************************************************************************/

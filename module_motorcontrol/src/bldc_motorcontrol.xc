/**
 * @file comm_loop_server.xc
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <bldc_motorcontrol.h>
#include <pwm_service_client.h>
#include <a4935.h>
#include <sine_table_big.h>
#include <sine_table_foc.h>
#include <foc_interface.h>
#include <foc_utilities.h>
#include <print.h>
#include <xscope.h>

static void commutation_init_to_zero(chanend c_pwm_ctrl, t_pwm_control & pwm_ctrl)
{
    unsigned int pwm[3] = {0, 0, 0};  // PWM OFF (break mode; short all phases)
    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

[[combinable]]
void bldc_loop(HallConfig hall_config, QEIConfig qei_config,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[4],
                            chanend c_pwm_ctrl,
                            FetDriverPorts &fet_driver_ports,
                            MotorcontrolConfig &motorcontrol_config)
{
    const unsigned t_delay = 300*USEC_FAST;
    timer t;
    unsigned int ts;
    t_pwm_control pwm_ctrl;
    int check_fet;
    int calib_flag = 0;
    int init_state = INIT_BUSY;

    unsigned int pwm[3] = { 0, 0, 0 };
    int angle_pwm = 0;
    int angle = 0;
    int voltage = 0;
    int pwm_half = PWM_MAX_VALUE>>1;
    int max_count_per_hall, angle_offset;

    if (!isnull(i_hall)) {
        if(!isnull(i_qei))
            max_count_per_hall = qei_config.ticks_resolution * QEI_CHANGES_PER_TICK /hall_config.pole_pairs;
        angle_offset = (4096 / 6) / (2 * hall_config.pole_pairs);
    }

    int fw_flag = 0;
    int bw_flag = 0;

    int shutdown = 0; //Disable FETS
    int sensor_select = motorcontrol_config.commutation_sensor;

    int notification = MOTCTRL_NTF_EMPTY;

    commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

    // enable watchdog
    t :> ts;
    t when timerafter (ts + 250000*4):> ts; /* FIXME: replace with constant */
    i_watchdog.start();

    t :> ts;
    t when timerafter (ts + t_delay) :> ts;

    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast)){
        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
        t when timerafter (ts + t_delay) :> ts;
    }

    if(!isnull(fet_driver_ports.p_coast)){
        fet_driver_ports.p_coast :> check_fet;
        init_state = check_fet;
    }
    else {
        init_state = 1;
    }

    while (1) {

  //      t_loop :> start_time;

        select {

            case t when timerafter(ts + USEC_FAST * motorcontrol_config.commutation_loop_period) :> ts: //XX kHz commutation loop
                if (calib_flag != 0) {
                    angle = 0;
                } else if (sensor_select == HALL_SENSOR) {
                    //hall only
                    angle = i_hall.get_hall_position();
                } else if (sensor_select == QEI_SENSOR && !isnull(i_qei)) {
                    { angle, fw_flag, bw_flag } = i_qei.get_qei_sync_position();
                    angle = (angle << 12) / max_count_per_hall;
                    if ((voltage >= 0 && fw_flag == 0) || (voltage < 0 && bw_flag == 0)) {
                        angle = i_hall.get_hall_position();
                    }
                } else if (sensor_select == BISS_SENSOR) {
                    angle = i_biss.get_biss_angle();
                }
                if (motorcontrol_config.polarity_type == INVERTED_POLARITY)
                    angle = 4096 - angle;

                if (shutdown == 1) {    /* stop PWM */
                    pwm[0] = -1;
                    pwm[1] = -1;
                    pwm[2] = -1;
                } else {
                    if (voltage >= 0) {
                        if (sensor_select == QEI_SENSOR ) {
                            angle_pwm = (angle >> 2) & 0x3ff; //512
                        } else {
                            angle_pwm = ((angle + motorcontrol_config.hall_offset[0]) >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half; // 6944 -- 6867range
                        angle_pwm = (angle_pwm + 341) & 0x3ff; /* +120 degrees (sine LUT size divided by 3) */
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                    } else { /* voltage < 0 */
                        if (sensor_select == QEI_SENSOR ) {
                            angle_pwm = (angle >> 2) & 0x3ff; //3100
                        } else {
                            angle_pwm = ((angle + motorcontrol_config.hall_offset[1]) >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 341) & 0x3ff;
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                    }
                }

                /* Limiting PWM values (and suppression of short pulses) is done in
                 * update_pwm_inv() */
                update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
                break;

            case i_motorcontrol[int i].get_notification() -> int out_notification:

                out_notification = notification;
                break;

            case i_motorcontrol[int i].set_voltage(int new_voltage):
                    if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
                        voltage = -new_voltage;
                    else
                        voltage = new_voltage;
                    break;

            case i_motorcontrol[int i].set_config(MotorcontrolConfig new_parameters):
                    motorcontrol_config = new_parameters;

                    notification = MOTCTRL_NTF_CONFIG_CHANGED;
                    // TODO: Use a constant for the number of interfaces
                    for (int i = 0; i < 4; i++) {
                        i_motorcontrol[i].notification();
                    }

                    sensor_select = motorcontrol_config.commutation_sensor;
                    break;

            case i_motorcontrol[int i].get_config() -> MotorcontrolConfig out_config:

                    out_config = motorcontrol_config;
                    break;

            case i_motorcontrol[int i].set_sensor(int new_sensor):
                    sensor_select = new_sensor;
                    motorcontrol_config.commutation_sensor = sensor_select;
                    break;

            case i_motorcontrol[int i].set_sensor_offset(int in_offset):
                    if (sensor_select == BISS_SENSOR ) {
                        BISSConfig out_biss_config = i_biss.get_biss_config();
                        out_biss_config.offset_electrical = in_offset;
                        i_biss.set_biss_config(out_biss_config);
                    }
                    break;

            case i_motorcontrol[int i].set_fets_state(int new_state):

                    if(new_state == 0){
                        shutdown = 1;
                    }else{
                        shutdown = 0;
                        voltage = 0;
                    }

                    break;

            case i_motorcontrol[int i].get_fets_state() -> int fets_state:
                    fets_state = !shutdown;
                    break;

            case i_motorcontrol[int i].check_busy() -> int state_return:
                    state_return = init_state;
                    break;

            case i_motorcontrol[int i].set_calib(int in_flag) -> int out_offset:
                    calib_flag = in_flag;
                    if (calib_flag == 1) {
                        motorcontrol_config.hall_offset[0] = 0;
                        motorcontrol_config.hall_offset[1] = 2048;
                    } else {
                        int calib_angle;
                        if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                            calib_angle = 1024;
                        else
                            calib_angle = 3072;
                        if (sensor_select == HALL_SENSOR) {
                            out_offset = (1024 - i_hall.get_hall_position()) & 4095;
                            //Hall has a low resolution so the offsets could need to be shifted by +/- 1/6 turn
                            if (motorcontrol_config.polarity_type == INVERTED_POLARITY) {
                                if (motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                                    motorcontrol_config.hall_offset[0] = (out_offset - 682) & 4095; // -1/6 turn
                                    motorcontrol_config.hall_offset[1] = (out_offset + 682) & 4095; // + half turn - 1/6 turn
                                } else {
                                    motorcontrol_config.hall_offset[1] = (out_offset - 682) & 4095;
                                    motorcontrol_config.hall_offset[0] = (out_offset + 682) & 4095;
                                }
                            } else {
                                if (motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                                    motorcontrol_config.hall_offset[0] = out_offset;
                                    motorcontrol_config.hall_offset[1] = (out_offset + 2731) & 4095; // + half turn + 1/6 turn
                                } else {
                                    motorcontrol_config.hall_offset[1] = out_offset;
                                    motorcontrol_config.hall_offset[0] = (out_offset + 2731) & 4095;
                                }
                            }
                        } else if (sensor_select == BISS_SENSOR) {
                            out_offset = i_biss.reset_biss_angle_electrical(calib_angle);// quarter turn
                        }
                    }
                    break;

            case i_motorcontrol[int i].set_all_parameters(HallConfig in_hall_config,
                                                                QEIConfig in_qei_config,
                                                                MotorcontrolConfig in_commutation_config):

                 //hall_config.pole_pairs = in_hall_config.pole_pairs;
                 qei_config.index_type = in_qei_config.index_type;
                 //qei_config.max_ticks_per_turn = in_qei_config.max_ticks_per_turn;
                 qei_config.ticks_resolution = in_qei_config.ticks_resolution;

                 motorcontrol_config.hall_offset[0] = in_commutation_config.hall_offset[0];
                 motorcontrol_config.hall_offset[1] = in_commutation_config.hall_offset[1];
                 motorcontrol_config.bldc_winding_type = in_commutation_config.bldc_winding_type;
                 //motorcontrol_config.angle_variance = (60 * 4096) / (hall_config.pole_pairs * 2 * 360);

                // if (hall_config.pole_pairs < 4) {
                //      motorcontrol_config.nominal_speed = nominal_speed * 4;
                //  } else if (hall_config.pole_pairs >= 4) {
                //      motorcontrol_config.nominal_speed = nominal_speed;
                //  }

                  voltage = 0;
                  if (!isnull(i_hall)) {
                      if(!isnull(i_qei))
                          max_count_per_hall = qei_config.ticks_resolution  * QEI_CHANGES_PER_TICK / hall_config.pole_pairs;
                      angle_offset = (4096 / 6) / (2 * hall_config.pole_pairs);
                  }
                  fw_flag = 0;
                  bw_flag = 0;

                    break;

            }

 //       t_loop :> end_time;
 //       printf("%i kHz\n", 250000/(end_time - start_time));
    }

}

#define def_BOOST_RANGE 32
#define defBOOST 128

#define defADD 65536
#define RAMP_UMOT 1000 // msec  50 - 1000
//#define PWM_MAX_LIMIT (PWM_MAX_VALUE - PWM_DEAD_TIME)

#define HALL_OFFSET 3500

void foc_loop( FetDriverPorts &fet_driver_ports, server interface foc_base i_foc,
                    chanend c_pwm_ctrl,  interface ADCInterface client ?i_adc,  interface HallInterface client ?i_hall, interface WatchdogInterface client i_watchdog)
{
    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    t_pwm_control pwm_ctrl;
    unsigned int pwm[3] = { 0, 0, 0 };
    const unsigned t_delay = 300*USEC_FAST;
    timer tx;
    unsigned ts;
    int check_fet;
    int init_state = INIT_BUSY;
    //unsigned loop_start_time;
    //unsigned loop_next_time;
    int umot_temp;
    int iTemp1;


    int     mmTheta=0;
    int     mmSinus=0;
    int     mmCosinus=0;

    //ADC
    int current_ph_b = 0, current_ph_c = 0;
    //Hall
    int angle_electrical = 0;
    unsigned hall_pin_state = 0;
    //FOC
    int clarke_alpha = 0;
    int clarke_beta = 0;
    int park_alpha = 0;
    int park_beta = 0;
    int angle_inv_park = 0;
    int field_new = 0;
    int torq_new = 0;
    int torq_period = 0;
    int torq_pt1 = 0;
    int field_mean = 0;
    int angle_current = 0;
    int vector_current = 0;
    int boost = defBOOST;
    int umot_motor = 0;
    int angle_pwm = 0;
    int speed_actual = 0;
    int velocity = 0;
    int angle_offset = HALL_OFFSET;//910
    int par_ramp_umot = RAMP_UMOT;//FixMe: this define should be a parameter
    int pwm_enabled = 0;
    int umot_out = 0;
    int q_value = 0;

    int iXX[64];
    int iCX[64];
    int filter_sum[8];

    //FOC Field control
    int field_e1 = 0;
    int field_e2 = 0;
    int field_out_p_part = 0;
    int field_out_i_part = 0;
    int par_field_kp = 8000;//16000
    int par_field_ki = 2000;
    int field_out = 0;
    int field_out1 = 0;
    int field_out2 = 0;
    unsigned start_time = 0, end_time = 0;


    printstr("\n*********************************************************");
    printstr("\n                F O C - L O O P                          \n");
    printstr("*********************************************************\n");


    //================ init PWM ===============================
    //ToDo: define it as function (see bldc_motorcontrol.xc)
//    unsigned pwm[3] = { 0, 0, 0 };
//    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, s_pwm_control);
//    update_pwm_inv(s_pwm_control, c_pwm_ctrl, pwm);
    commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

/* ToDo: this is an old initialization sequence. Check if ADC calibration can be done before initialization
    tx :> ts;
    tx when timerafter (ts + t_delay) :> ts;


    //=== init driver =============
    a4935_initialize(p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, A4935_BIT_PWML | A4935_BIT_PWMH);

    tx when timerafter (ts + t_delay) :> ts;

    watchdog_start(c_watchdog);

    calib_data I_calib;
    do_adc_calibration_ad7949(c_adc, I_calib);

    watchdog_enable_motors(c_watchdog);
*/

    // enable watchdog
    tx :> ts;
    tx when timerafter (ts + 250000*4):> ts; /* FIXME: replace with constant */
    i_watchdog.start();

    tx :> ts;
    tx when timerafter (ts + t_delay) :> ts;

    //=== init driver =============
    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast)){
        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
        tx when timerafter (ts + t_delay) :> ts;
    }

    if(!isnull(fet_driver_ports.p_coast)){
        fet_driver_ports.p_coast :> check_fet;
        init_state = check_fet;
    }
    else {
        init_state = 1;
    }

    pwm_enabled = 1;

//=================55,6 usec ============= F O C - L O O P =================================================================================
//FixMe: in reality it is 110 usec
//==========================================================================================================================================

    while(1)
    {

// ToDo implement a state machine
//        case MotorOff:  foc_motor_off(fp);
//                        stop_pwm(pwm, fp);
//                        zero_pwm(pwm, fp);
//                        set_field_zero(fp);

        tx :> start_time;
     select {
         case i_foc.set_q(int q_value_):
             q_value = q_value_;
             break;

         case i_foc.get_torque_actual() -> int torque_actual_:
             torque_actual_ = torq_pt1;
             break;

         default:
             break;
     }


    //====================== get phase currents ======================================
        {current_ph_b, current_ph_c} = i_adc.get_currents();//42 usec in triggered, 15 usec on request

        xscope_int(PHASE_B, current_ph_b);
        xscope_int(PHASE_C, current_ph_c);

    //====================== get hall sensor ticks, angle, and velocity ======================================
//FixMe what is measure_tick for? Why is it zero inside the loop? Is named as 'speed_clock' inside filters
 //   fp.measure_tick = 0;



        {hall_pin_state, angle_electrical, velocity} = i_hall.get_hall_pinstate_angle_velocity();//2 - 17 usec
        xscope_int(HALL_PINS, hall_pin_state);
        xscope_int(ANGLE_ELECTRICAL, angle_electrical);

 //       xscope_int(VELOCITY, velocity);


    //==========================  clarke transformation  ===========================================================================
    {clarke_alpha, clarke_beta} = clarke_transformation(current_ph_b, current_ph_c);

    xscope_int(CLARKE_ALPHA, clarke_alpha);
    xscope_int(CLARKE_BETA, clarke_beta);

    //========================== park transform ============================
    mmTheta = angle_electrical/4; //normalization: sine table is with 1024 base points, angle is 0 - 4095
    mmTheta &= 0x3FF;
    mmSinus  = sine_table_1024[mmTheta];         // sine( fp.Minp[mmTheta] );
    mmTheta = (256 - mmTheta);              // 90-fp.Minp[mmTheta]
    mmTheta &= 0x3FF;
    mmCosinus  = sine_table_1024[mmTheta];       // values from 0 to +/- 16384

    field_new  = (((clarke_alpha * mmCosinus )  /16384) + ((clarke_beta *  mmSinus ) /16384));
    torq_new   = (((clarke_beta  * mmCosinus )  /16384) - ((clarke_alpha * mmSinus ) /16384));

    xscope_int(FIELD_RAW, field_new);
    xscope_int(TORQUE_RAW, torq_new);

   //========================= filter torque  =======================================================================
    torq_period          = calc_mean_one_periode(iCX, iXX, torq_new, torq_period, hall_pin_state, 16);  // every cycle

    torq_pt1             = low_pass_pt1_filter(filter_sum, fftorque, 4,  torq_new);

    xscope_int(TORQUE_PERIOD, torq_period);
    xscope_int(TORQUE_PT1, torq_pt1);

   //========================= filter field ===========================================================

    field_mean           = calc_mean_one_periode(iCX, iXX, field_new, field_mean, hall_pin_state, 32);  // every cycle
    xscope_int(FIELD_MEAN, field_mean);
    xscope_int(FIELD_RAW, field_new);


    //================================= calculate iVectorCurrent ===============================================

    {iTemp1, angle_current}  = cartesian_to_polar_conversion( clarke_alpha, clarke_beta);

    vector_current    = calc_mean_one_periode(iCX, iXX, iTemp1, vector_current, hall_pin_state, 0);  // every cycle

//FixMe: Once per electrical rotation, but why? measure tick takes values 0x80 41 42 43 44 45
//    if(fp.measure_tick & 0x80)   field_control(fp);
//    if(hall_pin_state == 6)
        field_out = field_control(field_mean, field_e1, field_e2, q_value, field_out_p_part, field_out_i_part, par_field_kp, par_field_ki, field_out1, field_out2, filter_sum); //ToDo: analyze the condition

    xscope_int(FIELD_CONTROLLER_OUTPUT, field_out);
    //====================================================================================================

    //============== FOC INVERS PARK =====================================================================
    if(boost > 0) boost--;
    if(boost < 0) boost++;


    {park_alpha, park_beta }      = invers_park(mmSinus, mmCosinus, field_out, (q_value + boost/def_BOOST_RANGE));

    {umot_motor, angle_inv_park}  = cartesian_to_polar_conversion(park_alpha, park_beta);


     if(umot_motor > 4096) umot_motor = 4096;     // limit high
     if(umot_motor <   40) umot_motor = 0;        // limit low

  //   speed_actual = velocity;

     angle_pwm  =  adjust_angle_reference_pwm(angle_inv_park, angle_offset, hall_pin_state, speed_actual, q_value, filter_sum);


     //==================== umot_out follows umot_motor with a ramp ==========================
     umot_temp   = umot_motor * 65536;

     if(umot_out  < umot_temp)
     {
         umot_out += par_ramp_umot;
         if(umot_out > umot_temp) umot_out = umot_temp;
     }
     if(umot_out  > umot_temp)
     {
         umot_out -= par_ramp_umot;
         if(umot_out < umot_temp) umot_out = umot_temp;
     }
     //=======================================================================================



    //========== prepare PWM =================================================================

     if(pwm_enabled)
     {
         space_vector_pwm((umot_out/65536),  angle_pwm,  pwm_enabled, pwm);
     }



     /*till this point 55 usec loop period*/

     update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);  // !!!  must be the last function in this loop; 55 usec @ 18kHz config, why?

     tx :> end_time;

     xscope_int(CYCLE_TIME, (end_time - start_time)/250);
    }//============== end while(1) =====

}//======== end function foc_loop ================================================================


void space_vector_pwm(int umot, int angle, int pwm_on_off, unsigned pwmout[])
{
     if(pwm_on_off == 0)
     { pwmout[0]= 0;   pwmout[1]=0;   pwmout[2]=0;  return; }

     if(umot > 4096) umot = 4096;

     angle /= 4;     // 0 - 4095  => 0 - 1023
     angle &= 0x03ff;

     // pwm 13889 * 4 nsec = 55,6 microsec  18Khz
    // space vector table with 1024 base points   Umax = 6944
     pwmout[1] = ((SPACE_TABLE[angle]) * umot)/4096   + PWM_MAX_VALUE/2;  // PhaseB
     angle = (angle +341) & 0x3ff;
     pwmout[2] = ((SPACE_TABLE[angle])*  umot)/4096   + PWM_MAX_VALUE/2;  // PhaseC
     angle = (angle + 342) & 0x3ff;
     pwmout[0] = ((SPACE_TABLE[angle])* umot)/4096    + PWM_MAX_VALUE/2;  // PhaseA


     if(pwmout[0] < PWM_MIN_LIMIT)      pwmout[0] = 0;
     if(pwmout[1] < PWM_MIN_LIMIT)      pwmout[1] = 0;
     if(pwmout[2] < PWM_MIN_LIMIT)      pwmout[2] = 0;
}


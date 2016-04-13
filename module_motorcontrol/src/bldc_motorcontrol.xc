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
#include <stdlib.h>

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
                            interface AMSInterface client ?i_ams,
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
                } else if (sensor_select == AMS_SENSOR) {
                    angle = i_ams.get_ams_angle();
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

            case i_motorcontrol[int i].get_torque_actual() -> int torque_actual:
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

            case i_motorcontrol[int i].get_voltage() -> int out_voltage:
                    out_voltage =  voltage;
                    break;

            case i_motorcontrol[int i].set_torque(int torque_sp):
                    printstr("\n> ERROR: setting torque directly is not supported for SINE commutation, please check your motorcontrol configuration!");
                    break;

            case i_motorcontrol[int i].set_torque_max(int torque_max_):
                    printstr("\n> ERROR: controlling torque directly is not supported for SINE commutation, please check your motorcontrol configuration!");
                    break;

            case i_motorcontrol[int i].set_control(int in_flag):
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
                    } else if (sensor_select == AMS_SENSOR ) {
                        AMSConfig out_ams_config = i_ams.get_ams_config();
                        out_ams_config.offset = in_offset;
                        i_ams.set_ams_config(out_ams_config);
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
                        } else if (sensor_select == AMS_SENSOR) {
                            out_offset = i_ams.reset_ams_angle(calib_angle);// quarter turn
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
#define RAMP_UMOT 65536 // 1 - 65536 max

void foc_loop( FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                                        HallConfig hall_config, QEIConfig qei_config,
                                        interface MotorcontrolInterface server i_motorcontrol[4],
                    chanend c_pwm_ctrl, interface ADCInterface client ?i_adc,
                                        interface HallInterface client ?i_hall,
                                        interface QEIInterface client ?i_qei,
                                        interface BISSInterface client ?i_biss,
                                        interface AMSInterface client ?i_ams,
                                        interface WatchdogInterface client i_watchdog,
                                        interface BrakeInterface client ?i_brake)
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

    int notification = MOTCTRL_NTF_EMPTY;
    int shutdown = 0; //Disable FETS
    int sensor_select = motorcontrol_config.commutation_sensor;
    int calib_flag = 0;
    int field_control_flag = 1;
    int max_count_per_hall;
    int fw_flag = 0;
    int bw_flag = 0;

    int     mmTheta=0;
    int     mmSinus=0;
    int     mmCosinus=0;

    //ADC
    int current_ph_b = 0, current_ph_c = 0;
    int adc_a, adc_b;
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
    int field_lpf = 0;
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
    int angle_offset = 0;//ANGLE_OFFSET;//910
    int par_ramp_umot = RAMP_UMOT;//FixMe: this define should be a parameter
    int pwm_enabled = 0;
    int umot_out = 0;
    unsigned umot_init = 0;
    int q_value = 0;
    int q_direct_select = 0;
    int q_limit = 4096;

    int iXX[64];
    int iCX[64];
    int filter_sum[8];

    //FOC Field control
    int field_e1 = 0;
    int field_e2 = 0;
    int field_out_p_part = 0;
    int field_out_i_part = 0;
    int par_field_kp = 32000;//16000
    int par_field_ki = 0;//2000
    int field_out = 0;
    int field_out1 = 0;
    int field_out2 = 0;
    unsigned start_time = 0, end_time = 0;

    //FOC Torque control
    int target_torque = 0;
    int torque_max = 4096;//max default value
    int actual_torque = 0;
    int error_torque = 0, error_torque_previous = 0;
    int error_torque_integral = 0;
    int error_torque_derivative = 0;
    int torque_control_output = 0;
    int pid_denominator = 10000;
    int torque_offset = 0;
    int error_torque_integral_limit = 100000;
    int Kp_n = 8000, Ki_n = 1000, Kd_n = 10;

    { adc_a, adc_b } = i_adc.get_external_inputs();
//    torque_offset = adc_b - adc_a;
//    torque_offset = 0;

    //================ init PWM ===============================
    commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

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

    if(motorcontrol_config.commutation_loop_period < 110){
        printstr("\n > WARNING: FOC loop period defined too short, setting to 110 usec\n");
        motorcontrol_config.commutation_loop_period = 110;
    } else if (motorcontrol_config.commutation_loop_period > 110){
        printstr("\n > WARNING: FOC loop period defined too long, setting to 110 usec\n");
        motorcontrol_config.commutation_loop_period = 110;
    }
//=================55,6 usec ============= F O C - L O O P =================================================================================
//FixMe: Make proper synchronization with PWM and ADC. Currently loop period is 110 usec
//==========================================================================================================================================
    tx :> ts;
    while(1)
    {

     select {

         case tx when timerafter(ts) :> void:

             ts += USEC_FAST * motorcontrol_config.commutation_loop_period; //XX kHz commutation loop
             tx :> start_time;

             if (shutdown == 0) {
                 //====================== get phase currents ======================================
                 {current_ph_b, current_ph_c} = i_adc.get_currents();//42 usec in triggered, 15 usec on request
                 { adc_a, adc_b } = i_adc.get_external_inputs();
#ifdef USE_XSCOPE
                 xscope_int(PHASE_B, current_ph_b);
                 xscope_int(PHASE_C, current_ph_c);
#endif
                 //====================== get sensor angle and velocity ======================================

                 if (calib_flag == 0) {
                     if (sensor_select == HALL_SENSOR) {
                         //hall only
                         {hall_pin_state, angle_electrical, velocity} = i_hall.get_hall_pinstate_angle_velocity();//2 - 17 usec
                     } else if (sensor_select == QEI_SENSOR && !isnull(i_qei)) {
                         { angle_electrical, fw_flag, bw_flag } = i_qei.get_qei_sync_position();
                         angle_electrical = (angle_electrical << 12) / max_count_per_hall;
                         if ((q_value >= 0 && fw_flag == 0) || (q_value < 0 && bw_flag == 0)) {
                             angle_electrical = i_hall.get_hall_position();
                         }
                     } else if (sensor_select == BISS_SENSOR) {
                         angle_electrical = i_biss.get_biss_angle();
                         velocity = i_biss.get_biss_velocity();
                     } else if (sensor_select == AMS_SENSOR) {
                         //ToDo: preferably merge to a single interface call. Still currently does not introduce much of a delay.
                         angle_electrical = i_ams.get_ams_angle();
                         velocity = i_ams.get_ams_velocity();
                     }
                     else{
                         printstr("\n > FOC loop feedback sensor error\n");
                         exit(-1);
                     }

                     if (motorcontrol_config.polarity_type == INVERTED_POLARITY)
                         angle_electrical = 4096 - angle_electrical;

#ifdef USE_XSCOPE
                     xscope_int(ANGLE_ELECTRICAL, angle_electrical);

                     xscope_int(VELOCITY, velocity);
#endif

                     //==========================  clarke transformation  ===========================================================================
                     {clarke_alpha, clarke_beta} = clarke_transformation(current_ph_b, current_ph_c);

                     //========================== park transform ============================
                     mmTheta = angle_electrical/4; //normalization: sine table is with 1024 base points, angle is 0 - 4095
                     mmTheta &= 0x3FF;
                     mmSinus  = sine_table_1024[mmTheta];         // sine( fp.Minp[mmTheta] );
                     mmTheta = (256 - mmTheta);              // 90-fp.Minp[mmTheta]
                     mmTheta &= 0x3FF;
                     mmCosinus  = sine_table_1024[mmTheta];       // values from 0 to +/- 16384

                     field_new  = (((clarke_alpha * mmCosinus )  /16384) + ((clarke_beta *  mmSinus ) /16384));
                     torq_new   = (((clarke_beta  * mmCosinus )  /16384) - ((clarke_alpha * mmSinus ) /16384));

                     field_lpf = low_pass_pt1_filter(filter_sum, fffield, 4,  field_new);
#ifdef USE_XSCOPE
                     xscope_int(FIELD, field_lpf);
                     xscope_int(TORQUE_RAW, torq_new);
#endif
                     //========================= filter torque  =======================================================================
                     //ToDo: find a better filtering way
                     //torq_period = calc_mean_one_periode(iCX, iXX, torq_new, torq_period, hall_pin_state, 16);  // every cycle

                     torq_pt1             = low_pass_pt1_filter(filter_sum, fftorque, 4,  torq_new);
#ifdef USE_XSCOPE
                     xscope_int(TORQUE_PT1, torq_pt1);
#endif
                     //========================= filter field ===========================================================
                     //ToDo: find a better filtering way
                     //field_mean = calc_mean_one_periode(iCX, iXX, field_new, field_mean, hall_pin_state, 32);  // every cycle

                     //================================= calculate iVectorCurrent ===============================================
                     //ToDo: check if it is still needed, was used to check overcurrent (if(vector_current > (par_nominal_current * 2)){status_foc = stFaultOverCurrent20;// Motor stop})
                     //{iTemp1, angle_current}  = cartesian_to_polar_conversion( clarke_alpha, clarke_beta);
                     //vector_current    = calc_mean_one_periode(iCX, iXX, iTemp1, vector_current, hall_pin_state, 0);  // every cycle

                     torq_pt1 = (adc_b -adc_a - torque_offset)/4;
                     //==== PID Torque Controller ====
                     if(!q_direct_select){
//                         { adc_a, adc_b } = i_adc.get_external_inputs();
                         actual_torque = torq_pt1;
                         //compute the control output
                         error_torque = target_torque - actual_torque; // 350
                         error_torque_integral = error_torque_integral + error_torque;
                         error_torque_derivative = error_torque - error_torque_previous;

                         if (error_torque_integral > error_torque_integral_limit) {
                             error_torque_integral = error_torque_integral_limit;
                         } else if (error_torque_integral < -error_torque_integral_limit) {
                             error_torque_integral = -error_torque_integral_limit;
                         }

                         torque_control_output = (Kp_n * error_torque) +
                                 (Ki_n * error_torque_integral) +
                                 (Kd_n * error_torque_derivative);

                         torque_control_output /= pid_denominator;

                         error_torque_previous = error_torque;

                         //limit q (voltage) output
                         if (torque_control_output > q_limit) {
                             torque_control_output = q_limit;
                         }else if (torque_control_output < -q_limit) {
                             torque_control_output = -q_limit;
                         }

                         q_value = torque_control_output;
                     }
                     //===========================
#ifdef USE_XSCOPE
                     xscope_int(Q_VALUE, q_value);
                     xscope_int(TORQUE_SP, target_torque);
#endif
                     //==== Field Controller ====
                     if (field_control_flag == 1)
                         field_out = field_control(field_lpf, field_e1, field_e2, q_value, field_out_p_part, field_out_i_part, par_field_kp, par_field_ki, field_out1, field_out2, filter_sum); //ToDo: analyze the condition
                     else
                         field_out = 0;
#ifdef USE_XSCOPE
                     xscope_int(FIELD_CONTROLLER_OUTPUT, field_out);
#endif
                     //====================================================================================================

                     //============== FOC INVERS PARK =====================================================================
                     if(boost > 0) boost--;
                     if(boost < 0) boost++;


                     {park_alpha, park_beta }      = invers_park(mmSinus, mmCosinus, field_out, (q_value + boost/def_BOOST_RANGE));

                     {umot_motor, angle_inv_park}  = cartesian_to_polar_conversion(park_alpha, park_beta);


                     if(umot_motor > 4096) umot_motor = 4096;     // limit high
                     if(umot_motor <   40) umot_motor = 0;        // limit low

                     //speed_actual = velocity;

                     angle_pwm  =  adjust_angle_reference_pwm(angle_inv_park, motorcontrol_config.hall_offset[0], hall_pin_state, speed_actual, q_value, filter_sum, sensor_select);

                     //========== prepare PWM =================================================================

                 } else {//offset calibration
                     umot_motor = 500;
                     angle_pwm = 0;
                 }

                 space_vector_pwm(umot_motor,  angle_pwm,  pwm_enabled, pwm);
             } else {    /* stop PWM */
                 pwm[0] = -1;
                 pwm[1] = -1;
                 pwm[2] = -1;
             }

             /*till this point 55 usec loop period*/

             update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);  // !!!  must be the last function in this loop; 55 usec @ 18kHz config, why?

              tx :> end_time;
#ifdef USE_XSCOPE
              xscope_int(CYCLE_TIME, (end_time - start_time)/250);
#endif
             break;

         case i_motorcontrol[int i].set_voltage(int q_value_):
             q_direct_select = 1;
             pwm_enabled = 1;
             if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
                 q_value = -q_value_;
             else
                 q_value = q_value_;

             if (q_value > q_limit) //limit q (voltage)
                 q_value = q_limit;
             else if (q_value < -q_limit)
                 q_value = -q_limit;

             error_torque_previous = 0;
             error_torque_integral = 0;
             break;

         case i_motorcontrol[int i].get_voltage() -> int out_voltage:
                 out_voltage =  q_value;
                 break;

         case i_motorcontrol[int i].set_torque(int torque_sp):
             q_direct_select = 0;
             field_control_flag = 1;
             if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
                 torque_sp = -torque_sp;
             if ((target_torque ^ torque_sp) < 0) { //target torque sign changed, reset torque control
                 error_torque_previous = 0;
                 error_torque_integral = 0;
             }
             target_torque = torque_sp;
             //limit torque
             if(target_torque > torque_max)
                 target_torque = torque_max;
             else if (target_torque < -torque_max)
                 target_torque = -torque_max;
             pwm_enabled = 1;
             break;

         case i_motorcontrol[int i].set_torque_max(int torque_max_):
             torque_max = torque_max_;
             if(torque_max > 4096) torque_max = 4096;
             if(target_torque > torque_max)
                 target_torque = torque_max;
             else if (target_torque < -torque_max)
                 target_torque = -torque_max;
             break;

         case i_motorcontrol[int i].get_torque_actual() -> int torque_actual:
                 torque_actual = torq_pt1;
             break;

         case i_motorcontrol[int i].get_notification() -> int out_notification:

             out_notification = notification;
             break;

         case i_motorcontrol[int i].set_config(MotorcontrolConfig new_parameters):
                 if (motorcontrol_config.bldc_winding_type != new_parameters.bldc_winding_type)
                     q_value = -q_value;
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
                 } else if (sensor_select == AMS_SENSOR ) {
                     AMSConfig out_ams_config = i_ams.get_ams_config();
                     out_ams_config.offset = in_offset;
                     i_ams.set_ams_config(out_ams_config);
                 }
                 break;

         case i_motorcontrol[int i].set_fets_state(int new_state):

                 if(new_state == 0){
                     shutdown = 1;
                     pwm_enabled = 0;
                     if (!isnull(i_brake))
                         i_brake.set_brake(0);
                 }else{
                     shutdown = 0;
                     q_value = 0;
                     q_direct_select = 1;
                     error_torque_previous = 0;
                     error_torque_integral = 0;
                     if (!isnull(i_brake)) {
                         i_brake.set_brake(1);
                         delay_ticks(100*MSEC_STD);
                     }
                 }

                 break;

         case i_motorcontrol[int i].get_fets_state() -> int fets_state:
                 fets_state = !shutdown;
                 break;

         case i_motorcontrol[int i].check_busy() -> int state_return:
                 state_return = init_state;
                 break;

         case i_motorcontrol[int i].set_control(int in_flag):
                 field_control_flag = in_flag;
                 if (field_control_flag == 0)
                     q_direct_select = 1;
                 break;

         case i_motorcontrol[int i].set_calib(int in_flag) -> int out_offset:
                 calib_flag = in_flag;
                 if (calib_flag == 0) {
                     int calib_angle;
                     if (motorcontrol_config.commutation_method == FOC) {
                         calib_angle = 2048;
                     } else if (motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                         calib_angle = 1024;
                     } else {
                         calib_angle = 3072;
                     }
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
                         //out_offset = i_biss.reset_biss_angle_electrical(calib_angle);// quarter turn
                         out_offset = (calib_angle - i_biss.get_biss_angle()) & 4095;
                         motorcontrol_config.hall_offset[0] = out_offset;
                         motorcontrol_config.hall_offset[1] = out_offset;
                     } else if (sensor_select == AMS_SENSOR) {
                         out_offset = i_ams.reset_ams_angle(calib_angle);// quarter turn
                         motorcontrol_config.hall_offset[0] = 0;
                         motorcontrol_config.hall_offset[1] = 0;
                     }
                 }
                 break;

         case i_motorcontrol[int i].set_all_parameters(HallConfig in_hall_config,
                                                             QEIConfig in_qei_config,
                                                             MotorcontrolConfig in_commutation_config):

                qei_config.index_type = in_qei_config.index_type;
                qei_config.ticks_resolution = in_qei_config.ticks_resolution;

                motorcontrol_config.hall_offset[0] = in_commutation_config.hall_offset[0];
                motorcontrol_config.hall_offset[1] = in_commutation_config.hall_offset[1];
                motorcontrol_config.bldc_winding_type = in_commutation_config.bldc_winding_type;
                //motorcontrol_config.angle_variance = (60 * 4096) / (hall_config.pole_pairs * 2 * 360);

                q_value = 0;
                if (!isnull(i_hall)) {
                   if(!isnull(i_qei))
                       max_count_per_hall = qei_config.ticks_resolution  * QEI_CHANGES_PER_TICK / hall_config.pole_pairs;
                   angle_offset = (4096 / 6) / (2 * hall_config.pole_pairs);
                }
                fw_flag = 0;
                bw_flag = 0;

             break;

     }

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


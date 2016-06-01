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
 void bldc_loop(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                interface MotorcontrolInterface server i_motorcontrol[4],
                chanend c_pwm_ctrl, interface ADCInterface client ?i_adc,
                client interface shared_memory_interface ?i_shared_memory,
                interface WatchdogInterface client i_watchdog,
                interface BrakeInterface client ?i_brake)
{
    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    t_pwm_control pwm_ctrl;
    const unsigned t_delay = 300*USEC_FAST;
    timer tx;
    unsigned ts;
    unsigned start_time = 0, end_time = 0;
    int check_fet;
    int init_state = INIT_BUSY;
    int notification = MOTCTRL_NTF_EMPTY;

    //ADC
    int current_ph_b = 0, current_ph_c = 0;
    int adc_a, adc_b;
    //Hall
    unsigned hall_pin_state = 0;
    //QEI
    int max_count_per_hall;
    int fw_flag = 0;
    int bw_flag = 0;
    //commutation
    int angle_electrical = 0;
    int angle_pwm = 0;
    int count = 0;
    if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
        motorcontrol_config.polarity_type = INVERTED_POLARITY;
    else
        motorcontrol_config.polarity_type = NORMAL_POLARITY;
    unsigned int pwm[3] = { 0, 0, 0 };
    int velocity = 0;
    int shutdown = 0; //Disable FETS
    int sensor_select = motorcontrol_config.commutation_sensor;
    int calib_flag = 0;
    int voltage_q = 0;
    int voltage_q_limit = 4096;
    //FOC
    int mmTheta=0;
    int mmSinus=0;
    int mmCosinus=0;
    int clarke_alpha = 0;
    int clarke_beta = 0;
    int park_alpha = 0;
    int park_beta = 0;
    int angle_inv_park = 0;
    int field_new = 0;
    int field_lpf = 0;
    int torq_new = 0;
//    int torq_period = 0;
    int torq_pt1 = 0;
//    int field_mean = 0;
    int boost = defBOOST;
    int umot_motor = 0;
    int pwm_enabled = 0;
    //SINE
    int pwm_half = PWM_MAX_VALUE>>1;

    //filters
//    int iXX[64];
//    int iCX[64];
    int filter_sum[8];

    //FOC Field control
    FieldControlParams field_control_params;
    field_control_params.kP = 32000;
    field_control_params.kI = 0;
    int field_out = 0;
    int field_control_flag = 1;

    //FOC Torque control
    int q_direct_select = 1;
    int target_torque = 0;
    int torque_max = 4096;//max default value
    int velocity_limit = 0x7fffffff; //max velocity during torque control
    int actual_torque = 0;
    int error_torque = 0, error_torque_previous = 0;
    int torque_control_output_previous = 0;
    int error_torque_integral = 0;
    int error_torque_derivative = 0;
    int pid_denominator = 10000;
    int error_torque_integral_limit = 100000;
    int k1 = 20, k2 = 100, k3 = 10;
    int Kp_n = 8000, Ki_n = 1000, Kd_n = 10;

    //torque sensor
    //int torque_offset = 0;
    //{ adc_a, adc_b } = i_adc.get_external_inputs();
    //torque_offset = adc_a - adc_b;
    //torque_offset = 0;

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

    if(motorcontrol_config.commutation_method == FOC){
        if(motorcontrol_config.commutation_loop_period < 110){
            printstr("\n > WARNING: FOC loop period defined too short, setting to 110 usec\n");
            motorcontrol_config.commutation_loop_period = 110;
        } else if (motorcontrol_config.commutation_loop_period > 110){
            printstr("\n > WARNING: FOC loop period defined too long, setting to 110 usec\n");
            motorcontrol_config.commutation_loop_period = 110;
        }
    }

    tx :> ts;

    i_adc.enable_overcurrent_protection();

    while(1) {
        select {

        case tx when timerafter(ts) :> start_time:

            ts += USEC_FAST * motorcontrol_config.commutation_loop_period; //XX kHz commutation loop

            if (shutdown == 0) { //commutation is enabled
                //====================== get sensor angle and velocity ======================================
                //from shared memory
                if (!isnull(i_shared_memory)) {
                    { angle_electrical, velocity, count } = i_shared_memory.get_angle_velocity_position();
                }

                //=========== SINE commutation ============//
                if(motorcontrol_config.commutation_method == SINE){
                    int voltage_pwm;
                    if (calib_flag == 0) {
                        //normalization: sine table is with 1024 base points, angle is 0 - 4095
                        //applying commutation offset and motor polarity
                        //we shift the angle by half a turn for negative voltage to correspond to the FOC commutation
                        if (motorcontrol_config.polarity_type != INVERTED_POLARITY) {
                            if (voltage_q >= 0) {
                                angle_pwm = ((angle_electrical + motorcontrol_config.hall_offset[0]) >> 2) & 1023;
                                voltage_pwm = voltage_q;
                            } else {
                                angle_pwm = ((angle_electrical + 2048 + motorcontrol_config.hall_offset[1]) >> 2) & 1023;
                                voltage_pwm = -voltage_q;
                            }
                        } else {
                            if (voltage_q >= 0) {
                                angle_pwm = ((4096 - angle_electrical - motorcontrol_config.hall_offset[0]) >> 2) & 1023;
                                voltage_pwm = voltage_q;
                            } else {
                                angle_pwm = ((4096 + 2048 - angle_electrical - motorcontrol_config.hall_offset[1]) >> 2) & 1023;
                                voltage_pwm = -voltage_q;
                            }
                        }
                    } else {
                        angle_pwm = 256; //1/4 turn (10 bit angle)
                        voltage_pwm = voltage_q;
                    }

                    pwm[1] = ((sine_third_expanded(angle_pwm)) * voltage_pwm) / pwm_half + pwm_half; // 6944 -- 6867range
                    angle_pwm = (angle_pwm + 341) & 0x3ff; /* +120 degrees (sine LUT size divided by 3) */
                    pwm[2] = ((sine_third_expanded(angle_pwm)) * voltage_pwm) / pwm_half + pwm_half;
                    angle_pwm = (angle_pwm + 342) & 0x3ff;
                    pwm[0] = ((sine_third_expanded(angle_pwm)) * voltage_pwm) / pwm_half + pwm_half;

                    /* Limiting PWM values (and suppression of short pulses) is done in
                     * update_pwm_inv() */

                    //=================55,6 usec ============= F O C - L O O P =================================================================================
                    //FixMe: Make proper synchronization with PWM and ADC. Currently loop period is 110 usec
                    //==========================================================================================================================================
                } else { //=========== FOC commutation ============//

                    //====================== get phase currents ======================================
                    {current_ph_b, current_ph_c} = i_adc.get_currents();//42 usec in triggered, 15 usec on request
                    { adc_a, adc_b } = i_adc.get_external_inputs();

                    if (calib_flag == 0) {
                        //normalization: sine table is with 1024 base points, angle is 0 - 4095
                        //applying commutation offset and motor polarity
                        //the cclk commutation offset is only used for Hall sensor
                        if (motorcontrol_config.polarity_type != INVERTED_POLARITY) {
                            if (sensor_select != HALL_SENSOR || voltage_q >= 0) {
                                mmTheta = ((angle_electrical + motorcontrol_config.hall_offset[0]) >> 2) & 1023;
                            } else {
                                mmTheta = ((angle_electrical + motorcontrol_config.hall_offset[1]) >> 2) & 1023;
                            }
                        } else {
                            if (sensor_select != HALL_SENSOR || voltage_q >= 0) {
                                mmTheta = ((4096 - angle_electrical - motorcontrol_config.hall_offset[0]) >> 2) & 1023;
                            } else {
                                mmTheta = ((4096 - angle_electrical - motorcontrol_config.hall_offset[1]) >> 2) & 1023;
                            }
                        }

                        //==========================  clarke transformation  ===========================================================================
                        {clarke_alpha, clarke_beta} = clarke_transformation(current_ph_b, current_ph_c);

                        //========================== park transform ============================
                        mmSinus   = sine_table_1024[mmTheta];                 //  sine( fp.Minp[mmTheta] )  values from 0 to +/- 16384
                        mmCosinus = sine_table_1024[(256 - mmTheta) & 1023];  //cosine(90-fp.Minp[mmTheta])

                        field_new  = (((clarke_alpha * mmCosinus )  /16384) + ((clarke_beta *  mmSinus ) /16384));
                        torq_new   = (((clarke_beta  * mmCosinus )  /16384) - ((clarke_alpha * mmSinus ) /16384));

                        field_lpf = low_pass_pt1_filter(filter_sum, fffield, 4,  field_new);
                        //========================= filter torque  =======================================================================
                        //ToDo: find a better filtering way
                        //torq_period = calc_mean_one_periode(iCX, iXX, torq_new, torq_period, hall_pin_state, 16);  // every cycle

                        torq_pt1             = low_pass_pt1_filter(filter_sum, fftorque, 4,  torq_new);
                        //                     torq_pt1 = (adc_b -adc_a + torque_offset)/4;
                        //========================= filter field ===========================================================
                        //ToDo: find a better filtering way
                        //field_mean = calc_mean_one_periode(iCX, iXX, field_new, field_mean, hall_pin_state, 32);  // every cycle

                        //================================= calculate iVectorCurrent ===============================================
                        //ToDo: check if it is still needed, was used to check overcurrent (if(vector_current > (par_nominal_current * 2)){status_foc = stFaultOverCurrent20;// Motor stop})
                        //{iTemp1, angle_current}  = cartesian_to_polar_conversion( clarke_alpha, clarke_beta);
                        //vector_current    = calc_mean_one_periode(iCX, iXX, iTemp1, vector_current, hall_pin_state, 0);  // every cycle

                        //==== PID Torque Controller ====
                        if(!q_direct_select){
                            //velocity limit
                            if (velocity > velocity_limit || velocity < -velocity_limit)
                                target_torque = 0;

                            actual_torque = torq_pt1;
                            error_torque = ((k1 * target_torque)/10) - actual_torque;
                            voltage_q = ((10000+k2) * error_torque) -
                                    ( 10000     * error_torque_previous) +
                                    ((10000-k3) * torque_control_output_previous);
                            voltage_q /= 10000;

                            //limit q (voltage) output
                            if (voltage_q > voltage_q_limit) {
                                voltage_q = voltage_q_limit;
                            }else if (voltage_q < -voltage_q_limit) {
                                voltage_q = -voltage_q_limit;
                            }

                            error_torque_previous = error_torque;
                            torque_control_output_previous = voltage_q;
                        }
                        //===========================
                        //==== Field Controller ====
                        if (field_control_flag == 1)
                            field_out = field_control(field_control_params, field_lpf, voltage_q, filter_sum); //ToDo: analyze the condition
                        else
                            field_out = 0;
                        //====================================================================================================

                        //============== FOC INVERS PARK =====================================================================
                        if(boost > 0) boost--;
                        if(boost < 0) boost++;

                        {park_alpha, park_beta }      = invers_park(mmSinus, mmCosinus, field_out, (voltage_q + boost/def_BOOST_RANGE));

                        {umot_motor, angle_inv_park}  = cartesian_to_polar_conversion(park_alpha, park_beta);


                        if(umot_motor > 4096) umot_motor = 4096;     // limit high
                        if(umot_motor <   40) umot_motor = 0;        // limit low

                        //FIXME we shift angle by 3/4 turn because it's the difference between angle_electrical and angle_inv_park
                        //angle_pwm  =  adjust_angle_reference_pwm(angle_inv_park, 0, hall_pin_state, velocity, voltage_q, filter_sum, sensor_select);
                        angle_pwm = (angle_inv_park + 3072) & 4095;

                        //========== prepare PWM =================================================================

                    } else {//offset calibration
                        umot_motor = voltage_q/2;
                        angle_pwm = 1024; //1/4 turn
                    }

                    space_vector_pwm(umot_motor,  angle_pwm,  pwm_enabled, pwm);
                } //end FOC commutation
            } else {    /* stop PWM */
                pwm[0] = -1;
                pwm[1] = -1;
                pwm[2] = -1;
            }

            /*till this point 55 usec loop period*/

            update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);  // !!!  must be the last function in this loop; 55 usec @ 18kHz config, why?

            tx :> end_time;

#ifdef USE_XSCOPE
            if (motorcontrol_config.commutation_method == FOC) {
                xscope_int(PHASE_B, current_ph_b);
                xscope_int(PHASE_C, current_ph_c);
                xscope_int(VELOCITY, velocity);
                xscope_int(FIELD, field_lpf);
                xscope_int(VOLTAGE, voltage_q);
                xscope_int(TORQUE, torq_pt1);
                xscope_int(TARGET_TORQUE, target_torque);
                xscope_int(ERROR_TORQUE, target_torque-actual_torque);
                xscope_int(ERROR_TORQUE_INTEGRAL, error_torque_integral);
                xscope_int(ANGLE_ELECTRICAL, mmTheta);
                xscope_int(ANGLE_PWM, angle_pwm>>2);
                xscope_int(CYCLE_TIME, (end_time - start_time)/USEC_FAST);
            }
#endif

            //FIXME: this is to prevent blocking by adding a microsecond before the next loop when the time is more than 110 us
            if (timeafter(end_time, ts))
                ts = end_time + USEC_FAST;
            break;

        case i_motorcontrol[int i].set_torque_control_enabled():
            break;

        case i_motorcontrol[int i].set_torque_control_disabled():
            break;

        case i_motorcontrol[int i].set_offset_detection_enabled():
            break;

        case i_motorcontrol[int i].set_offset_value(int offset_value):
            break;

        case i_motorcontrol[int i].set_voltage(int in_voltage_q):
                voltage_q = in_voltage_q;

                if (voltage_q > voltage_q_limit) //limit q (voltage)
                    voltage_q = voltage_q_limit;
                else if (voltage_q < -voltage_q_limit)
                    voltage_q = -voltage_q_limit;

                q_direct_select = 1;
                pwm_enabled = 1;
                error_torque_previous = 0;
                error_torque_integral = 0;
                break;

        case i_motorcontrol[int i].set_torque(int torque_sp):
                q_direct_select = 0;
                field_control_flag = 1;
                target_torque = torque_sp;
                //limit torque
                if(target_torque > torque_max)
                    target_torque = torque_max;
                else if (target_torque < -torque_max)
                    target_torque = -torque_max;
                pwm_enabled = 1;
                break;

        case i_motorcontrol[int i].set_torque_pid(int Kp, int Ki, int Kd) -> { int out_Kp, int out_Ki, int out_Kd }:
                if (Kp >= 0)
                    Kp_n = Kp;
                if (Ki >= 0)
                    Ki_n = Ki;
                if (Kd >= 0)
                    Kd_n = Kd;
                out_Kp = Kp_n;
                out_Ki = Ki_n;
                out_Kd = Kd_n;
                if ((Kp+Ki+Kd) > -3) { //at least on coeff changed
                    error_torque_previous = 0;
                    error_torque_integral = 0;
                }
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
                //FixMe: implement separated inteface call for the torque sensor if needed
//                torque_actual = torq_pt1;
                torque_actual = adc_b - adc_a;
                break;

        case i_motorcontrol[int i].get_velocity_actual() -> int velocity_actual:
                //ToDo: Implement!
                velocity_actual = velocity;
                break;

        case i_motorcontrol[int i].get_position_actual() -> int position_actual:
                //ToDo: Implement!
                position_actual = count;
                break;

        case i_motorcontrol[int i].get_field() -> int out_field:
                out_field = field_lpf;
                break;

        case i_motorcontrol[int i].get_notification() -> int out_notification:

                out_notification = notification;
                break;

        case i_motorcontrol[int i].set_config(MotorcontrolConfig new_parameters):
                motorcontrol_config = new_parameters;
                if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
                    motorcontrol_config.polarity_type = INVERTED_POLARITY;
                else
                    motorcontrol_config.polarity_type = NORMAL_POLARITY;

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

        case i_motorcontrol[int i].set_sensor_offset(int in_offset) -> int out_offset:
                break;

        case i_motorcontrol[int i].set_fets_state(int new_state):

                if(new_state == 0){
                    shutdown = 1;
                    pwm_enabled = 0;
                    if (!isnull(i_brake))
                        i_brake.set_brake(0);
                }else{
                    shutdown = 0;
                    voltage_q = 0;
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
                if (calib_flag != 0) {
                    motorcontrol_config.hall_offset[0] = 0;
                    motorcontrol_config.hall_offset[1] = 0;
                }
                break;

        case i_motorcontrol[int i].restart_watchdog():
                i_watchdog.start();
                delay_milliseconds(1250);
                i_watchdog.start();
                break;

        }

    }//============== end while(1) =====

}//======== end function foc_loop ================================================================


void space_vector_pwm(int umot, int angle, int pwm_on_off, unsigned pwmout[])
{
    if (pwm_on_off == 0) {
        pwmout[0]= 0;
        pwmout[1]= 0;
        pwmout[2]= 0;
        return;
    }

    if (umot > 4096)
        umot = 4096;

    angle = (angle >> 2) & 1023; // 0 - 4095  => 0 - 1023

    // pwm 13889 * 4 nsec = 55,6 microsec  18Khz
    // space vector table with 1024 base points   Umax = 6944
    pwmout[1] = ((SPACE_TABLE[angle]) * umot)/4096 + PWM_MAX_VALUE/2;  // PhaseB
    angle = (angle + 341) & 0x3ff;
    pwmout[2] = ((SPACE_TABLE[angle]) * umot)/4096 + PWM_MAX_VALUE/2;  // PhaseC
    angle = (angle + 342) & 0x3ff;
    pwmout[0] = ((SPACE_TABLE[angle]) * umot)/4096 + PWM_MAX_VALUE/2;  // PhaseA

    if(pwmout[0] < PWM_MIN_LIMIT)
        pwmout[0] = 0;
    if(pwmout[1] < PWM_MIN_LIMIT)
        pwmout[1] = 0;
    if(pwmout[2] < PWM_MIN_LIMIT)
        pwmout[2] = 0;
}


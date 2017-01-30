/*
 * demo_adc.xc
 *
 *  Created on: Jul 13, 2017
 *      Author: Synapticon GmbH
 */

#include <demo_adc.h>
//#include <stdio.h>
//#include <ctype.h>


void demo_adc(interface ADCInterface client i_adc)
{
    //
    //    int recieve_torque_vdc_ib_ic=0;
    //    int update_pwm=0;
    //    int sync_inc=0;
    //
    //    motor_parameters mp;
    //    control_variables cv;

    timer t;
    //    timer t2;

    //    int reset_counter=0;
    //
    //    int reset_faults_counter=0;
    //    int safe_torque_off_counter=0;

    unsigned time=0;
    //    unsigned time2=0;
    //    unsigned ts;

    //    int true_licence =1;

    //    //proper task startup
    //    t :> ts;
    //    t when timerafter (ts + (5000*20*250)) :> void;



    //    while(i_shared_memory.status()!=ACTIVE);
    //    while(i_watchdog.status()!=ACTIVE);
    while(i_adc.status()!=ACTIVE);
    //    while(i_update_pwm.status()!=ACTIVE);


    //    cv.pwm_max = 13000;
    //    cv.pwm_min = 1500;
    //    cv.fault_code = NO_FAULT;

    //    if(ifm_tile_usec==250)
    //    {
    //        sync_inc= 16384;
    //        cv.pwm_max = 13000;
    //        cv.pwm_min = 1500;
    //        recieve_torque_vdc_ib_ic=7400;
    //        update_pwm=14800;
    //
    //        //Set freq to 250MHz (always needed for proper timing)
    //        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    //    }
    //    else if(ifm_tile_usec==100)
    //    {
    //        sync_inc= 8192;
    //        cv.pwm_max = 7000;
    //        cv.pwm_min = 600;
    //        recieve_torque_vdc_ib_ic=3700;
    //        update_pwm=7577;
    //    }
    //    else if (ifm_tile_usec!=100 && ifm_tile_usec!=250)
    //    {
    //        cv.fault_code = WRONG_REF_CLK_FRQ;
    //    }
    //
    //    initialize_parameters(cv, mp, motorcontrol_config);

    //    i_adc.set_protection_limits( motorcontrol_config.protection_limit_over_current,
    //            motorcontrol_config.current_ratio,
    //            motorcontrol_config.protection_limit_over_voltage,
    //            motorcontrol_config.protection_limit_under_voltage);

    //    if(motorcontrol_config.licence != 0)
    //    {
    //        true_licence = 0;
    //        cv.fault_code = WRONG_LICENCE;
    //    }

    int i_b=0, i_c=0, v_dc=0, torque=0, fault_code=0;
    int period=10000;

    t :> time;
    while (1)
    {
        select
        {
        case t when timerafter(time) :> void:

        {i_b, i_c, v_dc, torque, fault_code} = i_adc.get_all_measurements();


        xscope_int(I_B, i_b);
        xscope_int(I_C, i_c);
        xscope_int(V_DC, v_dc);
        xscope_int(TORQUE, torque);
        xscope_int(FAULT_CODE, fault_code);

            //            t2 :> time2;
            //
            //            select
            //            {
            //            case i_motorcontrol[int i].reset_faults():
            //                    reset_faults_counter=1;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_brake_status(int brake_status):
            //                    cv.brake_on = brake_status;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_torque_control_enabled():
            //                    initialize_control(cv);
            //                    cv.safe_torque_off_mode=0;
            //                    cv.pwm_on  =1;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_torque_control_disabled():
            //                    cv.pwm_on  =0;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_offset_detection_enabled():
            //                    if(cv.all_offset_computations_done==1)
            //                    {
            //                        initialize_parameters(cv, mp, motorcontrol_config);
            //                        initialize_control(cv);
            //                        cv.brake_on=1;
            //                        cv.offset=0;
            //                        cv.offset_detection=1;
            //                        cv.offset_computed =0;
            //                        cv.transition_found=0;
            //                        cv.angle_state_1_computed=0;
            //                        cv.angle_state_2_computed=0;
            //                        cv.angle_state_3_computed=0;
            //                        cv.angle_state_4_computed=0;
            //                        cv.angle_state_5_computed=0;
            //                        cv.angle_state_6_computed=0;
            //                        cv.angle_state_7_computed=0;
            //                        cv.hall_state_1_angle = 0;
            //                        cv.hall_state_2_angle = 0;
            //                        cv.hall_state_3_angle = 0;
            //                        cv.hall_state_4_angle = 0;
            //                        cv.hall_state_5_angle = 0;
            //                        cv.hall_state_6_angle = 0;
            //
            //                        cv.all_offset_computations_done=0;
            //                        cv.safe_torque_off_mode=0;
            //                        cv.pwm_on  =1;
            //                    }
            //                    break;
            //
            //            case i_motorcontrol[int i].set_safe_torque_off_enabled():
            //                    safe_torque_off_counter=1;
            //                    break;
            //
            //            case i_motorcontrol[int i].get_sensor_polarity_state() -> int proper_sensor_polarity:
            //                    proper_sensor_polarity = cv.proper_sensor_polarity;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_offset_value(int offset_value):
            //                    cv.offset = offset_value;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_torque(int torque_sp):
            //
            //                    if(torque_sp>  motorcontrol_config.max_torque ) torque_sp= motorcontrol_config.max_torque;
            //                    if(torque_sp<(-motorcontrol_config.max_torque)) torque_sp=-motorcontrol_config.max_torque;
            //
            //                    if(torque_sp>( 7090000/cv.pd.ref_torque_gain)) torque_sp= 7090000/cv.pd.ref_torque_gain;
            //                    if(torque_sp<(-7090000/cv.pd.ref_torque_gain)) torque_sp=-7090000/cv.pd.ref_torque_gain;
            //
            //                    if(motorcontrol_config.polarity_type==  1 )
            //                        cv.pd.received_torque =  torque_sp*cv.pd.ref_torque_gain;
            //                    if(motorcontrol_config.polarity_type==(-1))
            //                        cv.pd.received_torque = -torque_sp*cv.pd.ref_torque_gain;
            //                    cv.pd.received_torque /= 1000;
            //                    break;
            //
            //
            //            case i_motorcontrol[int i].set_config(MotorcontrolConfig new_parameters):
            //                    motorcontrol_config=new_parameters;
            //                    initialize_parameters(cv, mp, motorcontrol_config);
            //                    initialize_control(cv);
            //                    break;
            //
            //            case i_motorcontrol[int i].get_config() -> MotorcontrolConfig out_config:
            //                    out_config = motorcontrol_config;
            //                    break;
            //
            //            case i_motorcontrol[int i].set_calib(int in_flag) -> int out_offset:
            //                    if(cv.all_offset_computations_done==0) out_offset=-1;
            //                    if(cv.all_offset_computations_done==1) out_offset=cv.offset;
            //                    break;
            //
            //            case i_motorcontrol[int i].get_velocity_actual()-> int velocity_actual:
            //                    velocity_actual = cv.speed;
            //                    break;
            //
            //            case i_motorcontrol[int i].get_position_actual()-> int position_actual:
            //                    position_actual = cv.position;
            //                    break;
            //
            //            case i_motorcontrol[int i].update_upstream_control_data()-> UpstreamControlData upstream_control_data:
            //
            //                    //updating the variables which should be sent to higher controlling levels:
            //                    if(motorcontrol_config.polarity_type==  1 )
            //                    {
            //                        upstream_control_data.computed_torque  =  cv.pd.torque_actual;
            //                        upstream_control_data.torque_set = cv.pd.received_torque;
            //                    }
            //                    if(motorcontrol_config.polarity_type==(-1))
            //                    {
            //                        upstream_control_data.computed_torque  = -cv.pd.torque_actual;
            //                        upstream_control_data.torque_set = -cv.pd.received_torque;
            //                    }
            //                    upstream_control_data.torque_set *= cv.pd.torque_gain;
            //                    upstream_control_data.torque_set /= cv.pd.ref_torque_gain;
            //
            //                    upstream_control_data.computed_torque *= cv.pd.torque_gain;
            //                    upstream_control_data.computed_torque /= cv.pd.ref_torque_gain;
            //
            //                    upstream_control_data.V_dc = cv.V_dc;
            //
            //                    upstream_control_data.angle = cv.angle_actual;
            //                    upstream_control_data.position = cv.position;
            //                    upstream_control_data.velocity = cv.speed;
            //                    upstream_control_data.sensor_torque = cv.pd.torque_sensor;
            //
            //                    upstream_control_data.error_status = cv.fault_code;
            //
            //                    break;
            //
            //            default:
            //                break;
            //            }
            //
            //            if(reset_faults_counter>0)
            //            {
            //                switch (reset_faults_counter)
            //                {
            //                case 1:
            //                    cv.pwm_on = 0;  //stop pwm
            //                    break;
            //
            //                case 10:
            //                    cv.safe_torque_off_mode=1;
            //                    cv.pwm_on = 0;  //stop pwm
            //                    i_update_pwm.safe_torque_off_enabled(); // go to safe_torque_off mode
            //                    break;
            //
            //                case 20:
            //                    cv.brake_on = 0; //lock the brakes
            //                    break;
            //
            //                case 30:
            //                    i_adc.reset_faults(); //reset fault state
            //                    break;
            //
            //                case 40:
            //                    cv.pd.received_torque = 0;
            //                    cv.pd.torque_set = 0;
            //                    initialize_control(cv);
            //                    cv.offset_detection=0;
            //                    cv.offset_computed =1;
            //                    initialize_parameters(cv, mp, motorcontrol_config);
            //                    cv.safe_torque_off_mode=1;
            //                    break;
            //                }
            //
            //                if(reset_faults_counter==150)
            //                {
            //                    reset_faults_counter=0;
            //                    reset_counter++;
            //                }
            //
            //                if(reset_counter==50)
            //                {
            //                    reset_counter=0;
            //                    reset_faults_counter=-1;
            //                    if(motorcontrol_config.licence != 0)
            //                    {
            //                        true_licence = 0;
            //                        cv.fault_code = WRONG_LICENCE;
            //                    }
            //                    else
            //                    {
            //                        true_licence = 1;
            //                    }
            //                }
            //
            //                reset_faults_counter++;
            //
            //            }
            //
            //
            //            if(safe_torque_off_counter>0)
            //            {
            //                switch (safe_torque_off_counter)
            //                {
            //
            //                case 1:
            //                    cv.pd.received_torque = 0;
            //                    cv.pd.torque_set = 0;
            //                    break;
            //
            //                case 15000:
            //                    cv.pwm_on = 0;  //stop pwm
            //                    break;
            //
            //                case 15001:
            //                    cv.safe_torque_off_mode=1;
            //                    break;
            //
            //                case 15002:
            //                    i_update_pwm.safe_torque_off_enabled();
            //                    break;
            //
            //                case 15003:
            //                    safe_torque_off_counter=-1;
            //                    break;
            //                }
            //                safe_torque_off_counter++;
            //            }
            //
            //            if(true_licence == 1)
            //            {
            //                cv.pd.torque_set = cv.pd.received_torque;
            //
            //                {cv.angle_actual, cv.speed, cv.position, cv.hall_state} = i_shared_memory.get_angle_velocity_position_hall();
            //
            //                cv.angle_actual += cv.offset;
            //                //cv.angle_actual = cv.angle_actual + (power_factor*cv.speed)/1000;
            //                if(cv.angle_actual>4095) cv.angle_actual -= 4096;
            //                if(cv.angle_actual<   0) cv.angle_actual += 4096;
            //
            //                //TODO offset detection should be implemented for final FOC code release
            //                offset_detection(cv, motorcontrol_config);
            //
            //                if (cv.all_offset_computations_done==1)
            //                {
            //                    if(cv.speed<20 && cv.speed>(-20) && motorcontrol_config.commutation_sensor==HALL_SENSOR && (cv.hall_state_1_angle+cv.hall_state_2_angle!=0))
            //                        start_up_correction(cv, motorcontrol_config);
            //
            //                    cv.mmTheta = cv.angle_actual/4;
            //                    cv.mmTheta &= 0x3FF;
            //                    cv.mmSinus  = sine_table_control[cv.mmTheta];         // sine( fp.Minp[mmTheta] );
            //                    cv.mmTheta = (256 - cv.mmTheta);              // 90-fp.Minp[mmTheta]
            //                    cv.mmTheta &= 0x3FF;
            //                    cv.mmCosinus  = sine_table_control[cv.mmTheta];       // values from 0 to +/- 16384
            //
            //                    regenerative_mode(cv, mp, motorcontrol_config);
            //                }
            //
            //                t2 when timerafter (time2 + recieve_torque_vdc_ib_ic) :> void;
            //
            //                {cv.I_b, cv.I_c, cv.V_dc, cv.pd.torque_sensor, cv.fault_code} = i_adc.get_all_measurements();
            //
            //
            //                transformations(cv);
            //                correct(cv, mp);
            //                disturbance_rejection(cv, mp);
            //                predict(cv, mp, motorcontrol_config);
            //                limit_output(cv, motorcontrol_config);
            //                if (cv.all_offset_computations_done==1) InversPark(cv);
            //                voltage_calculation(cv, mp);
            //
            //                t2 when timerafter (time2 + update_pwm) :> void;
            //                i_update_pwm.update_server_control_data(cv.pwm_values[0], cv.pwm_values[1], cv.pwm_values[2], cv.pwm_on, cv.brake_on, cv.safe_torque_off_mode);
            //            }

            time +=period;
            break;

        }// select
    } // while(1)

}

//int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
//{
//    printf("Sending offset_detection command ...\n");
//    i_motorcontrol.set_offset_detection_enabled();
//
//    while(i_motorcontrol.set_calib(0)==-1) delay_milliseconds(50);//wait until offset is detected
//
//    int offset=i_motorcontrol.set_calib(0);
//    printf("Detected offset is: %i\n", offset);
//    //    printf(">>  CHECK PROPER OFFSET POLARITY ...\n");
//    int proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();
//    if(proper_sensor_polarity == 1) {
//        printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
//    } else {
//        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
//    }
//    return offset;
//}


//void demo_torque_position_velocity_control(client interface PositionVelocityCtrlInterface i_position_control)
//{
//    delay_milliseconds(500);
//    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");
//
//    DownstreamControlData downstream_control_data;
//    PosVelocityControlConfig pos_velocity_ctrl_config;
//
//    MotorcontrolConfig motorcontrol_config;
//    int proper_sensor_polarity=0;
//    int offset=0;
//
//    int velocity_running = 0;
//    int velocity = 0;
//
//    int torque = 0;
//    int brake_flag = 0;
//    int period_us;     // torque generation period in micro-seconds
//    int pulse_counter; // number of generated pulses
//    int torque_control_flag = 0;
//
//    fflush(stdout);
//    //read and adjust the offset.
//    while (1)
//    {
//        char mode = '@';
//        char mode_2 = '@';
//        char mode_3 = '@';
//        char c;
//        int value = 0;
//        int sign = 1;
//        //reading user input.
//        while((c = getchar ()) != '\n')
//        {
//            if(isdigit(c)>0)
//            {
//                value *= 10;
//                value += c - '0';
//            }
//            else if (c == '-')
//            {
//                sign = -1;
//            }
//            else if (c != ' ')
//            {
//                if (mode == '@')
//                {
//                    mode = c;
//                }
//                else if (mode_2 == '@')
//                {
//                    mode_2 = c;
//                }
//                else
//                {
//                    mode_3 = c;
//                }
//            }
//        }
//        value *= sign;
//
//        switch(mode)
//        {
//        //position commands
//        case 'p':
//                downstream_control_data.offset_torque = 0;
//                downstream_control_data.position_cmd = value;
//                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                switch(mode_2)
//                {
//                //direct command with profile
//                case 'p':
//                        //bug: the first time after one p# command p0 doesn't use the profile; only the way back to zero
//                        pos_velocity_ctrl_config.enable_profiler = 1;
//                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                        printf("Go to %d with profile\n", value);
//                        i_position_control.update_control_data(downstream_control_data);
//                        break;
//                //step command (forward and backward)
//                case 's':
//                        switch(mode_3)
//                        {
//                        //with profile
//                        case 'p':
//                                pos_velocity_ctrl_config.enable_profiler = 1;
//                                printf("position cmd: %d to %d with profile\n", value, -value);
//                                break;
//                        //without profile
//                        default:
//                                pos_velocity_ctrl_config.enable_profiler = 0;
//                                printf("position cmd: %d to %d\n", value, -value);
//                                break;
//                        }
//                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                        downstream_control_data.offset_torque = 0;
//                        downstream_control_data.position_cmd = value;
//                        i_position_control.update_control_data(downstream_control_data);
//                        delay_milliseconds(1500);
//                        downstream_control_data.position_cmd = -value;
//                        i_position_control.update_control_data(downstream_control_data);
//                        delay_milliseconds(1500);
//                        downstream_control_data.position_cmd = 0;
//                        i_position_control.update_control_data(downstream_control_data);
//                        break;
//                //direct command
//                default:
//                        pos_velocity_ctrl_config.enable_profiler = 0;
//                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                        printf("Go to %d\n", value);
//                        i_position_control.update_control_data(downstream_control_data);
//                        break;
//                }
//                break;
//
//        //velocity commands
//        case 'v':
//                switch(mode_2)
//                {
//                //step command (forward and backward)
//                case 's':
//                        printf("velocity cmd: %d to %d\n", value, -value);
//                        downstream_control_data.offset_torque = 0;
//                        downstream_control_data.velocity_cmd = value;
//                        i_position_control.update_control_data(downstream_control_data);
//                        delay_milliseconds(1000);
//                        downstream_control_data.velocity_cmd = -value;
//                        i_position_control.update_control_data(downstream_control_data);
//                        delay_milliseconds(1000);
//                        downstream_control_data.velocity_cmd = 0;
//                        i_position_control.update_control_data(downstream_control_data);
//                        break;
//                //direct command
//                default:
//                        if(value==0)
//                            velocity_running = 0;
//                        else
//                            velocity_running = 1;
//                        downstream_control_data.offset_torque = 0;
//                        velocity = value;
//                        downstream_control_data.velocity_cmd = velocity;
//                        i_position_control.update_control_data(downstream_control_data);
//                        printf("set velocity %d\n", downstream_control_data.velocity_cmd);
//                        break;
//                }
//                break;
//
//        //enable and disable torque controller
//        case 't':
//                downstream_control_data.offset_torque = 0;
//                downstream_control_data.torque_cmd = value;
//                i_position_control.update_control_data(downstream_control_data);
//                printf("torque command %d milli-Nm\n", downstream_control_data.torque_cmd);
//                break;
//
//        //reverse torque
//        case 'r':
//                downstream_control_data.torque_cmd = -downstream_control_data.torque_cmd;
////                if(velocity_running)
////                {
////                    velocity = -velocity;
////                    downstream_control_data.offset_torque = 0;
////                    downstream_control_data.velocity_cmd = velocity;
////                    i_position_control.update_control_data(downstream_control_data);
////                }
//                i_position_control.update_control_data(downstream_control_data);
//                printf("torque command %d milli-Nm\n", downstream_control_data.torque_cmd);
//                break;
//
//        //pid coefficients
//        case 'k':
//                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                switch(mode_2)
//                {
//                case 'p': //position
//                        switch(mode_3)
//                        {
//                        case 'p':
//                                pos_velocity_ctrl_config.P_pos = value;
//                                break;
//                        case 'i':
//                                pos_velocity_ctrl_config.I_pos = value;
//                                break;
//                        case 'd':
//                                pos_velocity_ctrl_config.D_pos = value;
//                                break;
//                        case 'l':
//                                pos_velocity_ctrl_config.integral_limit_pos = value;
//                                break;
//                        case 'j':
//                                pos_velocity_ctrl_config.j = value;
//                                break;
//                        default:
//                                break;
//                        }
//                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                        pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                        printf("Kp:%d Ki:%d Kd:%d j%d i_lim:%d\n",
//                                pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos, pos_velocity_ctrl_config.D_pos,
//                                pos_velocity_ctrl_config.j, pos_velocity_ctrl_config.integral_limit_pos);
//                        break;
//
//                case 'v': //velocity
//                        switch(mode_3)
//                        {
//                        case 'p':
//                                pos_velocity_ctrl_config.P_velocity = value;
//                                break;
//                        case 'i':
//                                pos_velocity_ctrl_config.I_velocity = value;
//                                break;
//                        case 'd':
//                                pos_velocity_ctrl_config.D_velocity = value;
//                                break;
//                        case 'l':
//                                pos_velocity_ctrl_config.integral_limit_velocity = value;
//                                break;
//                        default:
//                                break;
//                        }
//                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                        pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                        printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
//                                pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
//                        break;
//
//                default:
//                        printf("kp->pos_ctrl ko->optimum_ctrl kv->vel_ctrl\n");
//                        break;
//                }
//
//                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                break;
//
//        //limits
//        case 'L':
//                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                switch(mode_2)
//                {
//                //max position limit
//                case 'p':
//                    switch(mode_3)
//                    {
//                    case 'u':
//                        pos_velocity_ctrl_config.max_pos = value;
//                        break;
//                    case 'l':
//                        pos_velocity_ctrl_config.min_pos = value;
//                        break;
//                    default:
//                        pos_velocity_ctrl_config.max_pos = value;
//                        pos_velocity_ctrl_config.min_pos = -value;
//                        break;
//                    }
//                    break;
//
//                //max velocity limit
//                case 'v':
//                        pos_velocity_ctrl_config.max_speed = value;
//                        break;
//
//                //max torque limit
//                case 't':
//                        pos_velocity_ctrl_config.max_torque = value;
//                        break;
//
//                default:
//                        break;
//                }
//                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                printf("pos_max:%d pos_min:%d v_max:%d torq_max:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.min_pos, pos_velocity_ctrl_config.max_speed,
//                        pos_velocity_ctrl_config.max_torque);
//                break;
//
//        //change direction/polarity of the movement in position/velocity control
//        case 'd':
//            pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//            if (pos_velocity_ctrl_config.polarity == -1)
//            {
//                pos_velocity_ctrl_config.polarity = 1;
//                printf("normal movement polarity\n");
//            }
//            else
//            {
//                pos_velocity_ctrl_config.polarity = -1;
//                printf("inverted movement polarity\n");
//            }
//            i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//            break;
//
//        //enable
//        case 'e':
//                switch(mode_2)
//                {
//                case 'p':
//                        if (value == 1)
//                        {
//                            i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
//                            printf("simple PID pos ctrl enabled\n");
//                        }
//                        else if (value == 2)
//                        {
//                            i_position_control.enable_position_ctrl(POS_PID_VELOCITY_CASCADED_CONTROLLER);
//                            printf("vel.-cascaded pos ctrl enabled\n");
//                        }
//                        else if (value == 3)
//                        {
//                            i_position_control.enable_position_ctrl(NL_POSITION_CONTROLLER);
//                            printf("Nonlinear pos ctrl enabled\n");
//                        }
//                        else
//                        {
//                            i_position_control.disable();
//                            printf("position ctrl disabled\n");
//                        }
//                        break;
//                case 'v':
//                        if (value == 1)
//                        {
//                            i_position_control.enable_velocity_ctrl();
//                            printf("velocity ctrl enabled\n");
//                        }
//                        else
//                        {
//                            i_position_control.disable();
//                            printf("velocity ctrl disabled\n");
//                        }
//                        break;
//                case 't':
//                        if (value == 1)
//                        {
//                            torque_control_flag = 1;
//                            i_position_control.enable_torque_ctrl();
//                            printf("torque ctrl enabled\n");
//                        }
//                        else
//                        {
//                            torque_control_flag = 0;
//                            i_position_control.disable();
//                            printf("torque ctrl disabled\n");
//                        }
//                        break;
//                default:
//                        printf("ep1->enable PID pos ctrl\n");
//                        printf("ep2->enable cascaded pos ctrl\n");
//                        printf("ep3->enable integral-optimum pos ctrl\n");
//                        printf("ev1->enable PID velocity ctrl\n");
//                        printf("et1->enable torque ctrl\n");
//                        break;
//                }
//                break;
//        //help
//        case 'h':
//                printf("p->set position\n");
//                printf("v->set veloctiy\n");
//                printf("k->set PIDs\n");
//                printf("L->set limits\n");
//                printf("e->enable controllers\n");
//                break;
//
//        //jerk limitation (profiler parameters)
//        case 'j':
//                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                switch(mode_2)
//                {
//                case 'a':
//                        pos_velocity_ctrl_config.max_acceleration_profiler = value;
//                        break;
//                case 'v':
//                        pos_velocity_ctrl_config.max_speed_profiler = value;
//                        break;
//                default:
//                        break;
//                }
//                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                printf("acceleration_max:%d velocity_max:%d\n",pos_velocity_ctrl_config.max_acceleration_profiler, pos_velocity_ctrl_config.max_speed_profiler);
//                break;
//
//        //auto offset tuning
//        case 'a':
//                printf("Sending offset_detection command ...\n");
//
//                motorcontrol_config = i_position_control.set_offset_detection_enabled();
//
//                if(motorcontrol_config.commutation_angle_offset == -1)
//                {
//                    printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
//                }
//                else
//                {
//                    motorcontrol_config = i_position_control.get_motorcontrol_config();
//                    printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
//
//                    printf("Detected offset is: %i\n", motorcontrol_config.commutation_angle_offset);
//
//                    if(motorcontrol_config.commutation_sensor==HALL_SENSOR)
//                    {
//                        printf("SET THE FOLLOWING CONSTANTS IN CASE OF LOW-QUALITY HALL SENSOR \n");
//                        printf("      hall_state_1_angle: %d\n", motorcontrol_config.hall_state_1);
//                        printf("      hall_state_2_angle: %d\n", motorcontrol_config.hall_state_2);
//                        printf("      hall_state_3_angle: %d\n", motorcontrol_config.hall_state_3);
//                        printf("      hall_state_4_angle: %d\n", motorcontrol_config.hall_state_4);
//                        printf("      hall_state_5_angle: %d\n", motorcontrol_config.hall_state_5);
//                        printf("      hall_state_6_angle: %d\n", motorcontrol_config.hall_state_6);
//                    }
//                }
//                break;
//
//        //set brake
//        case 'b':
//            switch(mode_2)
//            {
//            case 's':
//                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
//                pos_velocity_ctrl_config.special_brake_release = value;
//                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
//                break;
//            default:
//                if (brake_flag)
//                {
//                    brake_flag = 0;
//                    printf("Brake blocking\n");
//                }
//                else
//                {
//                    brake_flag = 1;
//                    printf("Brake released\n");
//                }
//                i_position_control.set_brake_status(brake_flag);
//                break;
//            }
//            break;
//
//        //set offset
//        case 'o':
//                motorcontrol_config = i_position_control.get_motorcontrol_config();
//                switch(mode_2)
//                {
//                //set offset
//                case 's':
//                    motorcontrol_config.commutation_angle_offset = value;
//                    i_position_control.set_motorcontrol_config(motorcontrol_config);
//                    printf("set offset to %d\n", motorcontrol_config.commutation_angle_offset);
//                    break;
//                //print offset
//                case 'p':
//                    printf("offset %d\n", motorcontrol_config.commutation_angle_offset);
//                    break;
//                }
//                break;
//
//        //disable controllers
//        default:
//                i_position_control.disable();
//                printf("controller disabled\n");
//                break;
//
//        }
//        delay_milliseconds(10);
//    }
//}

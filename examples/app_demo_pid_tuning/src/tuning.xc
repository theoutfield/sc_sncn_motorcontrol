/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int position_limit,
        interface MotorcontrolInterface client i_motorcontrol,
        interface PositionControlInterface client i_pos,
        interface TuningInterface client ?i_tuning,
        interface ADCInterface client ?i_adc)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");
    int offset = 0;
    int field_control_flag = 1;
    int torque_flag = 0;
    int input_voltage = 0;

    int velocity = 250, acceleration = 50;

    //set position limit
    if (position_limit && !isnull(i_tuning))
        i_tuning.set_limit(position_limit);

    fflush(stdout);
    //read and adjust the offset.
    while (1) {
        char mode = 0;
        char mode_2 = 0;
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            } else if (c == '-') {
                sign = -1;
            } else if (c != ' ') {
                if (mode == 0) {
                    mode = c;
                } else {
                    mode_2 = c;
                }
            }
        }
        switch(mode) {
        //go to position directly
        case 'd':
            if (!isnull(i_tuning))
                i_tuning.set_position_direct(value*sign);
            break;
        //toggle field controler
        case 'f':
            if (field_control_flag == 0) {
                field_control_flag = 1;
                printf("Field controler activated\n");
            } else {
                field_control_flag = 0;
                printf("Field and Torque controlers deactivated\n");
            }
            i_motorcontrol.set_control(field_control_flag);
            break;
        //pid
//        case 'k':
//            int Kp, Ki, Kd;
//            switch(mode_2) {
//            case 'p':
//                { Kp, Ki, Kd } = i_motorcontrol.set_torque_pid(value, -1, -1);
//                printf("Kp torque: %d\n", value);
//                break;
//            case 'i':
//                { Kp, Ki, Kd } = i_motorcontrol.set_torque_pid(-1, value, -1);
//                printf("Ki torque: %d\n", value);
//                break;
//            case 'd':
//                { Kp, Ki, Kd } = i_motorcontrol.set_torque_pid(-1, -1, value);
//                printf("Kd torque: %d\n", value);
//                break;
//            default:
//                { Kp, Ki, Kd } = i_motorcontrol.set_torque_pid(-1, -1, -1);
//                break;
//            }
//            printf("Kp_t: %d, Ki_t:%d, Kd_t: %d\n", Kp, Ki, Kd);
//            break;

        case 'k':
            int Kp, Ki, Kd;
            switch(mode_2) {
            case 'p':
                { Kp, Ki, Kd } = i_pos.set_pid_values(value, -1, -1);
                printf("Kp position: %d\n", value);
                break;
            case 'i':
                { Kp, Ki, Kd } = i_pos.set_pid_values(-1, value, -1);
                printf("Ki position: %d\n", value);
                break;
            case 'd':
                { Kp, Ki, Kd } = i_pos.set_pid_values(-1, -1, value);
                printf("Kd position: %d\n", value);
                break;
            default:
                { Kp, Ki, Kd } = i_pos.set_pid_values(-1, -1, -1);
                break;
            }
            printf("Kp_p: %d, Ki_p:%d, Kd_p: %d\n", Kp, Ki, Kd);
            break;
        //position limit
        case 'l':
            i_tuning.set_limit(value * sign);
            break;
        //go to position with profile
        case 'p':
            if (!isnull(i_tuning))
                i_tuning.set_position(value*sign, 20, 10);

            break;

        case 'q':
            switch(mode_2) {
                case 'v':
                    velocity = value * sign;
                    printf("Velocity: %d, acceleration: %d\n", velocity, acceleration);
                break;
                case 'a':
                    acceleration = value * sign;
                    printf("Velocity: %d, acceleration: %d\n", velocity, acceleration);
                break;
                case 'c':
                    printf("Velocity: %d, acceleration: %d\n", velocity, acceleration);
                    int act_pos;
                    if (!isnull(i_tuning))
                    {
                        act_pos = i_pos.get_position();
                        printf("Position: %d\n", act_pos);
                        i_tuning.set_position(act_pos + value*sign, velocity, acceleration);
//                        delay_milliseconds(20);
                        i_tuning.set_position(act_pos - value*sign, velocity, acceleration);
                        //delay_milliseconds(20);
                        //i_tuning.set_position(act_pos, velocity, acceleration);

                        i_tuning.set_position(act_pos + value*sign, velocity, acceleration);
//                        delay_milliseconds(20);
                        i_tuning.set_position(act_pos - value*sign, velocity, acceleration);
                        //delay_milliseconds(20);
                        i_tuning.set_position(act_pos, velocity, acceleration);
                    }
                    break;
                default:
                    printf("Velocity: %d, acceleration: %d\n", velocity, acceleration);
                    break;
            }
            break;

        //reverse voltage
        case 'r':
            input_voltage = -input_voltage;
            if (torque_flag) {
                i_motorcontrol.set_torque(input_voltage);
                if (!isnull(i_tuning))
                    i_tuning.set_torque(input_voltage);
                printf("torque %d\n", input_voltage);
            } else {
                i_motorcontrol.set_voltage(input_voltage);
                printf("voltage: %i\n", input_voltage);
            }
            break;
        //set torque limit
        case 't':
            if (mode_2 == 'l') {
                i_tuning.set_torque_limit(value);
                printf("torgue limit %d\n", value);
            } else {
                field_control_flag = 1;
                input_voltage = value * sign;
                i_motorcontrol.set_torque(input_voltage);
                torque_flag = 1;
                if (!isnull(i_tuning))
                    i_tuning.set_torque(input_voltage);
                printf("torque %d\n", input_voltage);
            }
            break;
        //set voltage
        default:
            i_tuning.set_position_direct(0x7fffffff);
            i_motorcontrol.set_fets_state(1);
            torque_flag = 0;
            if (value > 1500) value = 1500;
            input_voltage = value * sign;
            i_motorcontrol.set_voltage(input_voltage);
            printf("voltage: %i\n", input_voltage);
            break;
        }
        delay_milliseconds(10);
    }
}

int find_peak_current(interface ADCInterface client i_adc, int period, int times)
{   //find the peak current by sampling every [period] microseconds [times] times
    int peak_current = 0;
    for (int i=0; i<times; i++) {
        int current;
        {current, void} = i_adc.get_currents();
        if (current > peak_current)
            peak_current = current;
        else if (current < -peak_current)
            peak_current = -current;
        delay_microseconds(period);
    }
    return peak_current;
}


int auto_tuning_current(interface MotorcontrolInterface client i_motorcontrol, interface ADCInterface client i_adc, int input_voltage)
{
    int step = 2;
    int start_offset = 0;
    MotorcontrolConfig motorcontrol_config = i_motorcontrol.get_config();
    if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
        start_offset = motorcontrol_config.hall_offset[0];
    else
        start_offset = motorcontrol_config.hall_offset[1];
    //starting peak current and offset
    int best_offset = start_offset;
    int min_current = find_peak_current(i_adc, 1000, 200);
    int last_min_current;
    //search forward then backward
    for (int j=0; j<2; j++) {
        int offset = start_offset;
        do {
            last_min_current = min_current;
            for (int i=0; i<25; i++) {
                unsigned int pos_offset = (offset & 4095); //positive offset
                //update offset
                if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                    motorcontrol_config.hall_offset[0] = pos_offset;
                else
                    motorcontrol_config.hall_offset[1] = pos_offset;
                i_motorcontrol.set_config(motorcontrol_config);
                //find the peak current
                int peak_current = find_peak_current(i_adc, 1000, 200);
                //update minimum current and best offset
                if (peak_current < min_current) {
                    min_current = peak_current;
                    best_offset = pos_offset;
                }
                offset += step;
            }
        } while (min_current < last_min_current);
        step = -step;
    }
    if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
        motorcontrol_config.hall_offset[0] = best_offset;
    else
        motorcontrol_config.hall_offset[1] = best_offset;
    i_motorcontrol.set_config(motorcontrol_config);
    return best_offset;
}

static inline void update_offset(MotorcontrolConfig &motorcontrol_config, int voltage, int offset)
{
    if (motorcontrol_config.commutation_method == FOC || (voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
        motorcontrol_config.hall_offset[0] =  offset;
    else
        motorcontrol_config.hall_offset[1] =  offset;
}

[[combinable]]
 void tuning_service(interface TuningInterface server i_tuning, interface MotorcontrolInterface client i_motorcontrol,
                     interface ADCInterface client ?i_adc, interface PositionControlInterface client ?i_position_control,
                     interface HallInterface client ?i_hall, interface BISSInterface client ?i_biss, interface AMSInterface client ?i_ams,
                             interface CONTELECInterface client ?i_contelec)
{
    timer t;
    unsigned ts;
    t :> ts;
    MotorcontrolConfig motorcontrol_config = i_motorcontrol.get_config();
    int count = 0;
    int torque_offset = 0;
    int velocity = 0;
    int position_limit = 0;
    int position_limit_reached = 0;
    int print_position_limit = 0;
    int voltage = 0;
    int target_torque = 0;
    int current_sampling = 0;
    int peak_current = 0;
    int last_peak_current = 0;
    int phase_b, phase_c;
    int adc_a, adc_b;
    int enable_tuning = 0;
    int best_offset_pos, best_offset_neg, offset_pos, offset_neg, start_offset_pos, start_offset_neg;
    int min_current_pos, last_min_current_pos, min_current_neg, last_min_current_neg;
    int range_pos, range_neg, step_pos, step_neg;
    int tuning_done_pos, tuning_done_neg;

    if (!isnull(i_position_control)) {
        /* Initialise the position profile generator */
        ProfilerConfig profiler_config;
        profiler_config.polarity = POLARITY;
        profiler_config.max_position = MAX_POSITION_LIMIT;
        profiler_config.min_position = MIN_POSITION_LIMIT;
        profiler_config.max_velocity = MAX_VELOCITY;
        profiler_config.max_acceleration = MAX_ACCELERATION;
        profiler_config.max_deceleration = MAX_DECELERATION;
        init_position_profiler(profiler_config, i_position_control, i_hall, null, i_biss, i_ams, i_contelec);
    }

    //i_contelec.reset_contelec_position(0);
    //i_contelec.command_contelec(0x56, 0, 0);

//    if (!isnull(i_adc)) {
//        { adc_a, adc_b } = i_adc.get_external_inputs();
//        torque_offset = adc_b - adc_a;
//        torque_offset = 0;
//    }


    while(1) {
        select {
        case t when timerafter(ts) :> void:
            ts += USEC_STD * 1000;

            //get position and velocity
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR && !isnull(i_biss)) {
                velocity = i_biss.get_biss_velocity();
                { count, void, void } = i_biss.get_biss_position();
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR && !isnull(i_ams)) {
                velocity = i_ams.get_ams_velocity();
                { count, void } = i_ams.get_ams_position();
            } else if (motorcontrol_config.commutation_sensor == CONTELEC_SENSOR && !isnull(i_contelec)) {
                velocity = i_contelec.get_contelec_velocity();
                { count, void } = i_contelec.get_contelec_position();
            } else if (motorcontrol_config.commutation_sensor == HALL_SENSOR && !isnull(i_hall)) {
                count = i_hall.get_hall_position_absolute();
                velocity = i_hall.get_hall_velocity();
            }
//            xscope_int(VELOCITY, velocity);

//            //torque display
//            if (motorcontrol_config.commutation_method == FOC) {
//                int torque = i_motorcontrol.get_torque_actual();
//                int actual_voltage, error_torque_integral;
//                {actual_voltage, error_torque_integral} = i_motorcontrol.get_torque_control_out();
//                xscope_int(VOLTAGE, actual_voltage);
//                xscope_int(TORQUE, torque);
//                xscope_int(TARGET_TORQUE, target_torque);
//                xscope_int(ERROR_TORQUE, target_torque-torque);
//                xscope_int(ERROR_TORQUE_INTEGRAL, error_torque_integral);
//            }

//            if (!isnull(i_position_control)) {
////                xscope_int(TARGET_POSITION, i_position_control.get_target_position());
////                xscope_int(ACTUAL_POSITION, i_position_control.get_position());
////                xscope_int(COUNT, count);
//                //xscope_int(ACTUAL_POSITION, i_contelec.get_contelec_position);
//            }

            //postion limiter
            if (position_limit > 0) {
                if (count >= position_limit && velocity > 10) {
                    i_motorcontrol.set_voltage(0);
                    if (print_position_limit >= 0) {
                        print_position_limit = -1;
                        printf("up limit reached\n");
                    }
                    position_limit_reached = 1;
                } else if (count <= -position_limit && velocity < -10) {
                    i_motorcontrol.set_voltage(0);
                    if (print_position_limit <= 0) {
                        print_position_limit = 1;
                        printf("down limit reached\n");
                    }
                    position_limit_reached = 1;
                } else {
                    position_limit_reached = 0;
                }
            }
            break;

        case i_tuning.tune(int in_voltage):
            break;

        case i_tuning.set_position(int in_position, int velocity, int acc):
            if (!isnull(i_position_control)) {
                /* Set new target position for profile position control */
                set_profile_position(in_position, velocity, acc, acc, i_position_control);
                printf("Returned to %d\n", in_position);
                //i_position_control.disable_position_ctrl();
//                delay_milliseconds(500);
                //i_motorcontrol.set_fets_state(1);
            } else {
                printf("No position control\n");
            }
            break;

        case i_tuning.set_position_direct(int in_position):
            if (!isnull(i_position_control)) {
                if (in_position == 0x7fffffff) {
                    i_position_control.disable_position_ctrl();
                } else {
                    i_position_control.enable_position_ctrl();
                    i_position_control.set_position(in_position);
                    printf("Go to %d\n", in_position);
                }
            } else {
                printf("No position control\n");
            }
            break;

        case i_tuning.set_limit(int in_limit):
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR && !isnull(i_biss)) {
                i_biss.reset_biss_position(0);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR && !isnull(i_ams)) {
                i_ams.reset_ams_position(0);
            } else if (motorcontrol_config.commutation_sensor == CONTELEC_SENSOR && !isnull(i_contelec)) {
                i_contelec.reset_contelec_position(0);
            } else if (motorcontrol_config.commutation_sensor == HALL_SENSOR && !isnull(i_hall)) {
                i_hall.reset_hall_absolute_position(0);
            }
            if (in_limit < 0) {
                position_limit = in_limit;
                printf("Position limit disabled\n");
            } else if (in_limit > 0) {
                printf("Position limited to %d ticks around here\n", in_limit);
                position_limit = in_limit;
            } else {
                printf("Position limited around here\n");
            }
            break;

        case i_tuning.set_torque(int in_torque):
            target_torque = in_torque;
            break;

        case i_tuning.set_torque_limit(int in_torque_limit):
            if (!isnull(i_position_control)) {
                i_position_control.set_torque_limit(in_torque_limit);
            }
            break;
        }
    }
}

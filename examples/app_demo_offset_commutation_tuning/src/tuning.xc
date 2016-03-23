/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface TuningInterface client ?i_tuning, interface ADCInterface client ?i_adc)
{
    delay_seconds(1);
    printf(">>   SOMANET OFFSET TUNING SERVICE STARTING...\n");
    int offset = 0;
    int field_control_flag = 1;
    MotorcontrolConfig motorcontrol_config = i_commutation.get_config();
    if (motorcontrol_config.commutation_method == FOC) {
        field_control_flag = 0;
        i_commutation.set_control(field_control_flag);
        printf("FOC commutation\nField and Torque controllers deactivated\n");
    } else {
        printf("Sine commutation\n");
    }

    if (motorcontrol_config.commutation_sensor == HALL_SENSOR) {
        printf("Hall tuning, ");
    } else if (motorcontrol_config.commutation_sensor == BISS_SENSOR){
        offset = BISS_OFFSET_ELECTRICAL;
        printf("BiSS tuning, Sensor offset %d, ", offset);
    } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR){
        offset = AMS_OFFSET;
        printf("AMS tuning, Sensor offset %d, ", offset);
    }
    if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
        printf ( "Star winding, Polarity %d\noffset clk %d (for positive voltage)\noffset cclk %d (for negative voltage)\n", motorcontrol_config.polarity_type, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    else
        printf ("Delta winding, Polarity %d\noffset clk %d (for negative voltage)\noffset cclk %d (for positive voltage)\n", motorcontrol_config.polarity_type, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    printf("Enter a to start the auto sensor offset finding.\n");
    fflush(stdout);
    //read and adjust the offset.
    while (1) {
        char mode = 0;
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
            } else if (c != ' ')
                mode = c;
        }
        switch(mode) {
        //auto find offset
        case 'a':
            //stop the motor
            i_commutation.set_voltage(0);
            delay_milliseconds(500);
            i_commutation.set_calib(1);
            //set internal commutation voltage to 1000
            if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                i_commutation.set_voltage(1000);
            else
                i_commutation.set_voltage(-1000);
            //go to 1024 position (quarter turn)
            delay_milliseconds(500);
            offset = i_commutation.set_calib(0);
//            i_commutation. set_fets_state(0);
            //start turning the motor and print the offsets found
            i_commutation.set_voltage(input_voltage);
            motorcontrol_config = i_commutation.get_config();
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR || motorcontrol_config.commutation_sensor == AMS_SENSOR)
                printf("Sensor offset: %d, ", offset);
            printf("Voltage %d, Polarity %d\n", input_voltage, motorcontrol_config.polarity_type);
            if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("Now you can tune the offset clk: %d\n", motorcontrol_config.hall_offset[0]);
            else
                printf("Now you can tune the offset cclk: %d\n", motorcontrol_config.hall_offset[1]);
            printf("Enter c to start the auto tuning\nor enter a value to set the offset manually\n");
            break;
        //auto tune the offset by mesuring the current consumption
        case 'c':
//            if (!isnull(i_adc)) {
//                printf("Starting auto tuning...\n(This could take around 30 seconds)\n");
//                if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING)) {
//                    motorcontrol_config.hall_offset[0] = auto_tuning_current(i_commutation, i_adc, input_voltage);
////                    motorcontrol_config = i_commutation.get_config();
//                    printf("auto tuned offset clk: %d\n", motorcontrol_config.hall_offset[0]);
//                } else {
//                    motorcontrol_config.hall_offset[1] = auto_tuning_current(i_commutation, i_adc, input_voltage);
//                    printf("auto tuned offset cclk: %d\n", motorcontrol_config.hall_offset[1]);
//                }
//            } else {
//                printf("No adc service provided\n");
            //            }
//            if (input_voltage)
//                printf("Starting auto tuning...\n(This could take around 30 seconds)\n");
//            else
//                printf("Stop auto tuning...\n");
            i_tuning.tune(input_voltage);
            break;
        //set voltage
        case 'v':
            input_voltage = value * sign;
            i_commutation.set_voltage(input_voltage);
            if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, offset clk: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, offset cclk: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        //reverse voltage
        case 'r':
            input_voltage = -input_voltage;
            i_commutation.set_voltage(input_voltage);
            if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, offset clk: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, offset cclk: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        //reverse motor direction
        case 'd':
            if (motorcontrol_config.commutation_method == FOC) {
                if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                    motorcontrol_config.bldc_winding_type = DELTA_WINDING;
                else
                    motorcontrol_config.bldc_winding_type = STAR_WINDING;
                i_commutation.set_config(motorcontrol_config);
                printf("Reverse motor direction\n");
            } else {
                int temp = motorcontrol_config.hall_offset[0];
                motorcontrol_config.hall_offset[0] = motorcontrol_config.hall_offset[1];
                motorcontrol_config.hall_offset[1] = temp;
                i_commutation.set_config(motorcontrol_config);
                if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                    printf("Polarity %d, Voltage %d\noffset clk %d (for positive voltage)\noffset cclk %d (for negative voltage)\n", motorcontrol_config.polarity_type, input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
                else
                    printf("Polarity %d, Voltage %d\noffset clk %d (for negative voltage)\noffset cclk %d (for positive voltage)\n", motorcontrol_config.polarity_type, input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            }
            break;
        //go to 0 position
        case 'z':
            if (!isnull(i_tuning))
                i_tuning.set_position(0);
            break;
        case 'f':
            if (motorcontrol_config.commutation_method == FOC) {
                if (field_control_flag == 0) {
                    field_control_flag = 1;
                    printf("Field controler activated\n");
                } else {
                    field_control_flag = 0;
                    printf("Field and Torque controlers deactivated\n");
                }
                i_commutation.set_control(field_control_flag);
            }
            break;
        //set sensor offset
        case 's':
            offset = value;
            i_commutation.set_sensor_offset(offset);
            printf("Sensor offset: %d\n", offset);
            break;
        //print offsets, voltage and polarity
        case 'p':
            motorcontrol_config = i_commutation.get_config();
            if (motorcontrol_config.commutation_sensor == AMS_SENSOR || motorcontrol_config.commutation_sensor == BISS_SENSOR)
                printf("Sensor offset %d, ", offset);
            if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                printf("Polarity %d, Voltage %d\noffset clk %d (for positive voltage)\noffset cclk %d (for negative voltage)\n", motorcontrol_config.polarity_type, input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            else
                printf("Polarity %d, Voltage %d\noffset clk %d (for negative voltage)\noffset cclk %d (for positive voltage)\n", motorcontrol_config.polarity_type, input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            break;
        //reverse sensor direction
        case 'w':
            if (motorcontrol_config.polarity_type == NORMAL_POLARITY)
                motorcontrol_config.polarity_type = INVERTED_POLARITY;
            else
                motorcontrol_config.polarity_type = NORMAL_POLARITY;
            i_commutation.set_config(motorcontrol_config);
            printf("Polarity %d\n", motorcontrol_config.polarity_type);
            break;
        //position limit
        case 'l':
            if (value == 0)
                value = 0x7fffffff;
            i_tuning.set_limit(value);
            printf("Set position limit to %d\n", value);
            break;
        //set offset
        default:
            if (motorcontrol_config.commutation_method == FOC || (input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING)) {
                motorcontrol_config.hall_offset[0] = value;
                printf("offset clk: %d\n", value);
            } else {
                motorcontrol_config.hall_offset[1] = value;
                printf("offset cclk: %d\n", value);
            }
            i_commutation.set_config(motorcontrol_config);
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


int auto_tuning_current(interface MotorcontrolInterface client i_commutation, interface ADCInterface client i_adc, int input_voltage)
{
    int step = 2;
    int start_offset = 0;
    MotorcontrolConfig motorcontrol_config = i_commutation.get_config();
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
                i_commutation.set_config(motorcontrol_config);
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
    i_commutation.set_config(motorcontrol_config);
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
 void tuning_service(interface TuningInterface server i_tuning, interface MotorcontrolInterface client i_commutation,
                     interface ADCInterface client i_adc, interface PositionControlInterface client i_position_control,
                     interface BISSInterface client i_biss)
{
    timer t;
    unsigned ts;
    t :> ts;
    MotorcontrolConfig motorcontrol_config = i_commutation.get_config();
    int count = 0;
    int position_limit = 0x7fffffff;
    int position_limit_reached = 0;
    int voltage = 0;
    int current_sampling = 0;
    int peak_current = 0;
    int last_peak_current = 0;
    int phase_b, phase_c;
    int enable_tuning = 0;
    int best_offset_pos, best_offset_neg, offset_pos, offset_neg, start_offset_pos, start_offset_neg;
    int min_current_pos, last_min_current_pos, min_current_neg, last_min_current_neg;
    int range_pos, range_neg, step_pos, step_neg;
    int tuning_done_pos, tuning_done_neg;

    /* Initialise the position profile generator */
    ProfilerConfig profiler_config;
    profiler_config.polarity = POLARITY;
    profiler_config.max_position = MAX_POSITION_LIMIT;
    profiler_config.min_position = MIN_POSITION_LIMIT;
    profiler_config.max_velocity = MAX_VELOCITY;
    profiler_config.max_acceleration = MAX_ACCELERATION;
    profiler_config.max_deceleration = MAX_DECELERATION;
    init_position_profiler(profiler_config, i_position_control, null, null, i_biss, null);

    while(1) {
        select {
        case t when timerafter(ts) :> void:
            ts += USEC_STD * 1000;
            int velocity = i_biss.get_biss_velocity();
            { count, void, void } = i_biss.get_biss_position();

            if (count >= position_limit && velocity > 10) {
                position_limit_reached = 1;
                i_commutation.set_voltage(0);
                printf("up limit reached\n");
            } else if (count <= -position_limit && velocity < -10) {
                position_limit_reached = 1;
                i_commutation.set_voltage(0);
                printf("down limit reached\n");
            }

            {phase_b, phase_c} = i_adc.get_currents();

            if (current_sampling > 0) {
                current_sampling--;
                //find the peak current by sampling every [period] microseconds [times] times
                if (phase_b > peak_current)
                    peak_current = phase_b;
                else if (phase_b < -peak_current)
                    peak_current = -phase_b;
            }

            if (enable_tuning) {
                if (position_limit_reached) {
                    position_limit_reached = 0;
                    voltage = -voltage;
                    i_commutation.set_voltage(voltage);
                    delay_milliseconds(500);
                    printf("direction reversed\n");
                    current_sampling = 200;
                    peak_current = 0;
                }

                //new measured peak current
                if (current_sampling <= 0) {
                    if (voltage > 0 && tuning_done_pos == 0) {
                        //update min current and best offset
                        if (peak_current < min_current_pos) {
                            min_current_pos = peak_current;
                            best_offset_pos = (offset_pos & 4095);
                        }
                        if (range_pos <= 0) { //end of a range, check if the peak current is decreasing
                            printf("offset pos %d, %d -> %d\n", offset_pos, last_min_current_pos, min_current_pos);
                            if (min_current_pos >= last_min_current_pos) {
                                if (step_pos > 0) { //now search by decreasing the offset
                                    step_pos = -step_pos;
                                    offset_pos = start_offset_pos;
                                    printf("change direction, restart from %d\n", start_offset_pos);
                                } else { //offset pos tuning is done
                                    tuning_done_pos = 1;
                                    position_limit_reached = 1;
                                    printf("tuning pos done %d\n", best_offset_pos);
                                }
                            }
                            range_pos = 25;
                            last_min_current_pos = min_current_pos;
                        }
                        range_pos--;
                        offset_pos += step_pos;
                        update_offset(motorcontrol_config, voltage, (offset_pos & 4095));
                    } else if (voltage < 0 && tuning_done_neg == 0){ //negative voltage
                        //update min current and best offset
                        if (peak_current < min_current_neg) {
                            min_current_neg = peak_current;
                            best_offset_neg = (offset_neg & 4095);
                        }
                        if (range_neg <= 0) { //end of a range, check if the peak current is decreasing
                            printf("offset neg %d, %d -> %d\n", offset_neg, last_min_current_neg, min_current_neg);
                            if (min_current_neg >= last_min_current_neg) {
                                if (step_neg > 0) { //now search by decreasing the offset
                                    step_neg = -step_neg;
                                    offset_neg = start_offset_neg;
                                    printf("change direction, restart from %d\n", start_offset_neg);
                                } else { //offset neg tuning is done
                                    tuning_done_neg = 1;
                                    position_limit_reached = 1;
                                    printf("tuning neg done %d\n", best_offset_neg);
                                }
                            }
                            range_neg = 25;
                            last_min_current_neg = min_current_neg;
                        }
                        range_neg--;
                        offset_neg += step_neg;
                        update_offset(motorcontrol_config, voltage, (offset_neg & 4095));
                    }


                    if((tuning_done_pos + tuning_done_neg) >= 2) {//tuning is done
                        enable_tuning = 0;
                        if (motorcontrol_config.commutation_method == FOC || motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                            motorcontrol_config.hall_offset[0] = best_offset_pos;
                            motorcontrol_config.hall_offset[1] = best_offset_neg;
                        } else {
                            motorcontrol_config.hall_offset[1] = best_offset_pos;
                            motorcontrol_config.hall_offset[0] = best_offset_neg;
                        }
                        i_commutation.set_config(motorcontrol_config);
                        printf("Tuning done\nauto tuned offset clk: %d\nauto tuned offset cclk: %d\n", motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
                    } else {
                        last_peak_current = peak_current; //for displaying peak current
                        peak_current = 0;       // reset
                        current_sampling = 200; // current sampling
                        i_commutation.set_config(motorcontrol_config); //update offset
                    }
                } //end new measured peak current


            } //end tuning

            if (current_sampling <= 0) {
                last_peak_current = peak_current;
                current_sampling = 200;
                peak_current = 0;
            }
            xscope_int(PHASE_B, phase_b);
            xscope_int(PHASE_C, phase_c);
            xscope_int(PEAK_CURRENT, last_peak_current);
            xscope_int(VELOCITY, velocity);
            break;

        case i_tuning.tune(int in_voltage):
            voltage = in_voltage;
            if (voltage && enable_tuning == 0) {
                motorcontrol_config = i_commutation.get_config();
                if (motorcontrol_config.commutation_method == FOC) {
                    start_offset_pos = motorcontrol_config.hall_offset[0];
                    start_offset_neg = start_offset_pos;
                } else if (voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                    start_offset_pos = motorcontrol_config.hall_offset[0];
                    start_offset_neg = motorcontrol_config.hall_offset[1];
                } else {
                    start_offset_pos = motorcontrol_config.hall_offset[1];
                    start_offset_neg = motorcontrol_config.hall_offset[0];
                }
                best_offset_pos = start_offset_pos;
                offset_pos = start_offset_pos;
                best_offset_neg = start_offset_neg;
                offset_neg = start_offset_neg;
                enable_tuning = 1;
                last_min_current_pos = 10000;
                min_current_pos = 10000;
                last_min_current_neg = 10000;
                min_current_neg = 10000;
                peak_current = 0;
                current_sampling = 200;
                range_pos = 0;
                step_pos = 2;
                range_neg = 0;
                step_neg = 2;
                tuning_done_pos = 0;
                tuning_done_neg = 0;
                printf("Starting auto tuning...\n");
            } else {
                enable_tuning = 0;
                if (motorcontrol_config.commutation_method == FOC || motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                    motorcontrol_config.hall_offset[0] = best_offset_pos;
                    motorcontrol_config.hall_offset[1] = best_offset_neg;
                } else {
                    motorcontrol_config.hall_offset[1] = best_offset_pos;
                    motorcontrol_config.hall_offset[0] = best_offset_neg;
                }
                i_commutation.set_config(motorcontrol_config);
                printf("Tuning aborted!\nauto tuned offset clk: %d\nauto tuned offset cclk: %d\n", motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            }

            break;

        case i_tuning.set_position(int in_position):
            /* Set new target position for profile position control */
            set_profile_position(0, 200, 10, 10, i_position_control);
            printf("Returned to 0\n");
            i_position_control.disable_position_ctrl();
            i_commutation.set_fets_state(1);
            break;

        case i_tuning.set_limit(int in_limit):
            position_limit = in_limit;
            break;
        }
    }
}

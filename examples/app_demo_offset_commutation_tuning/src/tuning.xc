/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface BISSInterface client ?i_biss, interface AMSInterface client ?i_ams){

    delay_seconds(1);
    i_commutation.set_voltage(input_voltage);
    MotorcontrolConfig motorcontrol_config = i_commutation.get_config();
    BISSConfig biss_config;
    AMSConfig ams_config;

    if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
        printf ("Hall tuning. Voltage %d\nPlease enter an offset value different from %d, then press enter\n", input_voltage,
                (input_voltage > 0) ? ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset[0] : motorcontrol_config.hall_offset[1]) : ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset[1] : motorcontrol_config.hall_offset[0])  );
    else if (motorcontrol_config.commutation_sensor == BISS_SENSOR){
        biss_config = i_biss.get_biss_config();
        printf ("BiSS tuning. Voltage %d\nPlease enter an offset value different from %d, then press enter\n", input_voltage, biss_config.offset_electrical);
    } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR){
        ams_config = i_ams.get_ams_config();
        if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
            printf ("AMS tuning, Star winding, Voltage %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
        else
            printf ("AMS tuning, Delta winding, Voltage %d, offset cclk %d (positive voltage), offset clk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[1], motorcontrol_config.hall_offset[0]);
    }
    fflush(stdout);
    //read and adjust the offset
    char mode = 0;
    while (1) {
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
            } else
                mode = c;
        }
        if (motorcontrol_config.commutation_sensor == BISS_SENSOR || motorcontrol_config.commutation_sensor == AMS_SENSOR){
            switch(mode) {
            case 'a': //auto
                unsigned int offset;
                i_commutation.set_voltage(0);
                delay_milliseconds(500);
                if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                    i_commutation.set_voltage(1000);
                else
                    i_commutation.set_voltage(-1000);
                if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                    i_biss.set_biss_calib(1);
                    i_commutation.set_voltage(1000);
                    delay_milliseconds(200);
                    offset = i_biss.reset_biss_angle_electrical(2731);
                    i_biss.set_biss_calib(0);
                    printf("auto offset: %d\n", offset);
                } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                    motorcontrol_config.hall_offset[0] = 0;
                    i_commutation.set_config(motorcontrol_config);
                    i_ams.set_ams_calib(1);
                    delay_milliseconds(200);
                    motorcontrol_config.hall_offset[0] = (2731 - i_ams.set_ams_calib(0)) & 4095;
                    motorcontrol_config.hall_offset[1] = (2731 + motorcontrol_config.hall_offset[0]) & 4095;
                    i_commutation.set_config(motorcontrol_config);
                    printf("offset clk: %d, offset cclk: %d\n", motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
                }
                i_commutation.set_voltage(input_voltage);
                mode = 0;
                break;
            case 'v': //set voltage
                input_voltage = value * sign;
                i_commutation.set_voltage(input_voltage);
                printf("voltage: %i\n", input_voltage);
                mode = 0;
                break;
            case 'r': //reverse voltage
                input_voltage = -input_voltage;
                i_commutation.set_voltage(input_voltage);
                printf("voltage: %i\n", input_voltage);
                mode = 0;
                break;
            default: //set offset
                if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                    printf("offset: %d\n", value);
                    biss_config.offset_electrical = value;
                    i_biss.set_biss_config(biss_config);
                } else {
                    if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING)) {
                        motorcontrol_config.hall_offset[0] = value;
                        printf("offset clk: %d\n", value);
                    } else {
                        motorcontrol_config.hall_offset[1] = value;
                        printf("offset cclk: %d\n", value);
                    }
                    i_commutation.set_config(motorcontrol_config);
                }
                break;
            }
        } else { //please note for the delta winding type offset_clk and offset_cclk are flipped
            if (input_voltage > 0)
            {        //star winding
                if (motorcontrol_config.bldc_winding_type == 1) {
                    MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, HALL_SENSOR, {value, motorcontrol_config.hall_offset[1]}, motorcontrol_config.commutation_loop_period};
                    i_commutation.set_config(params);
                }
                else {
                    MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, HALL_SENSOR, {motorcontrol_config.hall_offset[0], value}, motorcontrol_config.commutation_loop_period};
                    i_commutation.set_config(params);
                }
            }
            else
            {
                if (motorcontrol_config.bldc_winding_type == 1){
                    MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, HALL_SENSOR, {motorcontrol_config.hall_offset[0], value}, motorcontrol_config.commutation_loop_period};
                    i_commutation.set_config(params);
                }
                else{
                    MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, HALL_SENSOR, {value, motorcontrol_config.hall_offset[1]}, motorcontrol_config.commutation_loop_period};
                    i_commutation.set_config(params);
                }
            }
        }

        delay_milliseconds(10);
    }
}

/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation){

    delay_seconds(1);
    i_commutation.set_voltage(input_voltage);
    MotorcontrolConfig motorcontrol_config = i_commutation.get_config();

    printf (" Please enter an offset value different from %d, then press enter\n",
            (input_voltage > 0) ? ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset[0] : motorcontrol_config.hall_offset[1]) : ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset[1] : motorcontrol_config.hall_offset[0])  );
    fflush(stdout);
    while (1) {
        char c;
        unsigned value = 0;
        //reading user input. Only positive integers are accepted
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            }
        }
        printf("setting %i\n", value);
        //please note for the delta winding type offset_clk and offset_cclk are flipped
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

        delay_milliseconds(10);
    }

}

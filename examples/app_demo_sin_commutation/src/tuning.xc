/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client commutation_interface){

    delay_seconds(1);
    commutation_interface.setVoltage(input_voltage);
    MotorcontrolConfig motorcontrol_config = commutation_interface.getConfig();

    printf (" Please enter an offset value different from %d, then press enter\n",
            (input_voltage > 0) ? ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset_clk : motorcontrol_config.hall_offset_cclk) : ((motorcontrol_config.bldc_winding_type == 1) ? motorcontrol_config.hall_offset_cclk : motorcontrol_config.hall_offset_clk)  );
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
                MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, value, motorcontrol_config.hall_offset_cclk, motorcontrol_config.commutation_loop_period};
                commutation_interface.setParameters(params);
            }
            else {
                MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, motorcontrol_config.hall_offset_clk, value, motorcontrol_config.commutation_loop_period};
                commutation_interface.setParameters(params);
            }
        }
        else
        {
            if (motorcontrol_config.bldc_winding_type == 1){
                MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, motorcontrol_config.hall_offset_clk, value, motorcontrol_config.commutation_loop_period};
                commutation_interface.setParameters(params);
            }
            else{
                MotorcontrolConfig params = {BLDC_MOTOR, motorcontrol_config.bldc_winding_type, value, motorcontrol_config.hall_offset_cclk, motorcontrol_config.commutation_loop_period};
                commutation_interface.setParameters(params);
            }
        }

        delay_milliseconds(10);
    }

}

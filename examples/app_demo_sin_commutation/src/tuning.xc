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

    printf (" Please enter an offset value different from %d, then press enter\n",
            (input_voltage > 0) ? ((WINDING_TYPE == 1) ? COMMUTATION_OFFSET_CLK : COMMUTATION_OFFSET_CCLK) : ((WINDING_TYPE == 1) ? COMMUTATION_OFFSET_CCLK : COMMUTATION_OFFSET_CLK)  );
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
            if (WINDING_TYPE == 1) {
                MotorcontrolConfig params = {(60 * 4096) / (POLE_PAIRS * 2 * 360), MAX_NOMINAL_SPEED, 0, 0, value, COMMUTATION_OFFSET_CCLK, WINDING_TYPE};
                commutation_interface.setParameters(params);
            }
            else {
                MotorcontrolConfig params = {(60 * 4096) / (POLE_PAIRS * 2 * 360), MAX_NOMINAL_SPEED, 0, 0, COMMUTATION_OFFSET_CLK, value, WINDING_TYPE};
                commutation_interface.setParameters(params);
            }
        }
        else
        {
            if (WINDING_TYPE == 1){
                MotorcontrolConfig params = {(60 * 4096) / (POLE_PAIRS * 2 * 360), MAX_NOMINAL_SPEED, 0, 0, COMMUTATION_OFFSET_CLK, value, WINDING_TYPE};
                commutation_interface.setParameters(params);
            }
            else{
                MotorcontrolConfig params = {(60 * 4096) / (POLE_PAIRS * 2 * 360), MAX_NOMINAL_SPEED, 0, 0, value, COMMUTATION_OFFSET_CCLK, WINDING_TYPE};
                commutation_interface.setParameters(params);
            }
        }

        delay_milliseconds(10);
    }

}

/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void set_commutation_offset_clk(chanend c_signal, unsigned offset){
    c_signal <: COMMUTATION_CMD_SET_PARAMS;
    c_signal <: (60 * 4096) / (POLE_PAIRS * 2 * 360);
    c_signal <: MAX_NOMINAL_SPEED;
    c_signal <: offset;
    c_signal <: COMMUTATION_OFFSET_CCLK;
    c_signal <: WINDING_TYPE;

}

void set_commutation_offset_cclk(chanend c_signal, unsigned offset){
    c_signal <: COMMUTATION_CMD_SET_PARAMS;
    c_signal <: (60 * 4096) / (POLE_PAIRS * 2 * 360);
    c_signal <: MAX_NOMINAL_SPEED;
    c_signal <: COMMUTATION_OFFSET_CLK;
    c_signal <: offset;
    c_signal <: WINDING_TYPE;

}


void run_offset_tuning(int input_voltage, chanend c_commutation_p1, chanend c_commutation_p2){

    delay_seconds(1);
    set_commutation_sinusoidal(c_commutation_p1, input_voltage);

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
            if (WINDING_TYPE == 1) set_commutation_offset_clk(c_commutation_p2, value);//910
            else set_commutation_offset_cclk(c_commutation_p2, value);//2460
        }
        else
        {
            if (WINDING_TYPE == 1) set_commutation_offset_cclk(c_commutation_p2, value);//2460
            else set_commutation_offset_clk(c_commutation_p2, value);//910
        }

        delay_milliseconds(10);
    }

}

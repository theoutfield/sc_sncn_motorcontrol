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
    int offset_mechanical = 0;

    if (motorcontrol_config.commutation_sensor == HALL_SENSOR) {
        if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
            printf ("Hall tuning, Star winding, Voltage %d, Mechanical offset %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
        else
            printf ("Hall tuning, Delta winding, Voltage %d, Mechanical offset %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    } else if (motorcontrol_config.commutation_sensor == BISS_SENSOR){
        biss_config = i_biss.get_biss_config();
        offset_mechanical = biss_config.offset_electrical;
        printf ("BiSS tuning. Voltage %d\nPlease enter an offset value different from %d, then press enter\n", input_voltage, biss_config.offset_electrical);
    } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR){
        ams_config = i_ams.get_ams_config();
        offset_mechanical = ams_config.offset;
        if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
            printf ("AMS tuning, Star winding, Voltage %d, Mechanical offset %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, offset_mechanical, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
        else
            printf ("AMS tuning, Delta winding, Voltage %d, Mechanical offset %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, offset_mechanical, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    }
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
            } else
                mode = c;
        }
        switch(mode) {
        case 'a': //auto
            i_commutation.set_voltage(0);
            delay_milliseconds(500);
            //set internal voltage to 1000
            if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                i_commutation.set_voltage(1000);
            else
                i_commutation.set_voltage(-1000);
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                i_biss.set_biss_calib(1);
                i_commutation.set_voltage(1000);
                delay_milliseconds(200);
                offset_mechanical = i_biss.reset_biss_angle_electrical(2731);
                i_biss.set_biss_calib(0);
                printf("auto offset: %d\n", offset_mechanical);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                motorcontrol_config.hall_offset[0] = 0;
                motorcontrol_config.hall_offset[1] = 2731; //FIXME: find the right value
                i_commutation.set_config(motorcontrol_config);
                i_ams.set_ams_calib(1);
                delay_milliseconds(200);
                //offset_mechanical = i_ams.reset_ams_angle((2731 - motorcontrol_config.hall_offset[0]) & 4095);
                //offset_mechanical = i_ams.reset_ams_angle(4096 - motorcontrol_config.hall_offset[0]);
                //offset_mechanical = i_ams.reset_ams_angle(0);
                offset_mechanical = i_ams.reset_ams_angle(2731); //FIXME: find the right value
                i_ams.set_ams_calib(0);
                printf("offset mechanical: %d, offset clk: %d, offset cclk: %d\n", offset_mechanical, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            }
            i_commutation.set_voltage(input_voltage);
            break;
        case 'v': //set voltage
            input_voltage = value * sign;
            i_commutation.set_voltage(input_voltage);
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, current clk offset: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, current cclk offset: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        case 'r': //reverse voltage
            input_voltage = -input_voltage;
            i_commutation.set_voltage(input_voltage);
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, current clk offset: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, current cclk offset: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        case 'm': //set mechanical offset
            offset_mechanical = value;
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                biss_config.offset_electrical = offset_mechanical;
                i_biss.set_biss_config(biss_config);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                ams_config.offset = offset_mechanical;
                i_ams.set_ams_config(ams_config);
            }
            printf("mechanical offset: %d\n", offset_mechanical);
            break;
        case 'p': //print
            if (motorcontrol_config.commutation_sensor == AMS_SENSOR || motorcontrol_config.commutation_sensor == BISS_SENSOR){
                if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                    printf ("Voltage %d, Mechanical offset %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, offset_mechanical, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
                else
                    printf ("Voltage %d, Mechanical offset %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, offset_mechanical, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            } else {
                if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                    printf ("Voltage %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
                else
                    printf ("Voltage %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            }
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
        delay_milliseconds(10);
    }
}

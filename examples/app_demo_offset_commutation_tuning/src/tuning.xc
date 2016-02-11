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
    int offset = 0;
    int polarity = -1;

    if (motorcontrol_config.commutation_sensor == HALL_SENSOR) {
        printf("Hall tuning, ");
    } else if (motorcontrol_config.commutation_sensor == BISS_SENSOR){
        biss_config = i_biss.get_biss_config();
        offset = biss_config.offset_electrical;
        polarity = biss_config.polarity;
        printf("BiSS tuning, Polarity %d, Sensor offset %d, ", polarity, offset);
    } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR){
        ams_config = i_ams.get_ams_config();
        offset = ams_config.offset;
        polarity = ams_config.polarity;
        printf("AMS tuning, Polarity %d, Sensor offset %d, ", polarity, offset);
    }
    if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
        printf ( "Star winding, Voltage %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    else
        printf ("Delta winding, Voltage %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
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
        case 'a': //auto find offset
            i_commutation.set_voltage(0);
            delay_milliseconds(500);
            //reset offsets: 0 and half a turn
            motorcontrol_config.hall_offset[0] = 0;
            motorcontrol_config.hall_offset[1] = 2048; // + half a turn
            i_commutation.set_config(motorcontrol_config);
            //set internal commutation voltage to 1000 for Start Winding or -1000 for Delta Winding
            i_commutation.set_voltage(1000);
            //go to 1024 position (quarter turn)
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                i_biss.set_biss_calib(1);
                delay_milliseconds(500);
                offset = i_biss.reset_biss_angle_electrical(1024);// quarter turn
                biss_config.offset_electrical = offset;
                i_biss.set_biss_calib(0);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                i_ams.set_ams_calib(1);
                delay_milliseconds(500);
                offset = i_ams.reset_ams_angle(1024);// quarter turn
                ams_config.offset = offset;
                i_ams.set_ams_calib(0);
            }
            i_commutation.set_voltage(input_voltage);
            printf("Sensor offset: %d, offset clk: %d, offset cclk: %d\n", offset, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
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
        case 's': //set sensor offset
            offset = value;
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                biss_config.offset_electrical = offset;
                i_biss.set_biss_config(biss_config);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                ams_config.offset = offset;
                i_ams.set_ams_config(ams_config);
            }
            printf("Sensor offset: %d\n", offset);
            break;
        case 'p': //print
            if (motorcontrol_config.commutation_sensor == AMS_SENSOR || motorcontrol_config.commutation_sensor == BISS_SENSOR)
                printf("Polarity %d, Sensor offset %d, ", polarity, offset);
            if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                printf ("Voltage %d, offset clk %d (positive voltage), offset cclk %d (negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            else
                printf ("Voltage %d, offset clk %d (negative voltage), offset cclk %d (positive voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            break;
        case 'd': //reverse sensor direction
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                if (biss_config.polarity == BISS_POLARITY_NORMAL)
                    biss_config.polarity = BISS_POLARITY_INVERTED;
                else
                    biss_config.polarity = BISS_POLARITY_NORMAL;
                polarity = biss_config.polarity;
                i_biss.set_biss_config(biss_config);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                if (ams_config.polarity == AMS_POLARITY_NORMAL)
                    ams_config.polarity = AMS_POLARITY_INVERTED;
                else
                    ams_config.polarity = AMS_POLARITY_NORMAL;
                polarity = ams_config.polarity;
                i_ams.set_ams_config(ams_config);
            }
            printf("Polarity %d\n", polarity);
            break;
        default: //set offset
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING)) {
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

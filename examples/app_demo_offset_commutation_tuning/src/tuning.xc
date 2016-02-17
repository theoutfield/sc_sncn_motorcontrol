/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface ADCInterface client i_adc,
                       interface BISSInterface client ?i_biss, interface AMSInterface client ?i_ams)
{
    delay_seconds(1);
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
        printf ( "Star winding\noffset clk %d (for positive voltage)\noffset cclk %d (for negative voltage)\n", motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
    else
        printf ("Delta winding\noffset clk %d (for negative voltage)\noffset cclk %d (for positive voltage)\n", motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
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
            //start turning the motor and print the offsets found
            i_commutation.set_voltage(input_voltage);
            motorcontrol_config = i_commutation.get_config();
            if (motorcontrol_config.commutation_sensor == BISS_SENSOR) {
                biss_config.offset_electrical = offset;
                printf("Sensor offset: %d, Polarity: %d, ", offset, polarity);
            } else if (motorcontrol_config.commutation_sensor == AMS_SENSOR) {
                ams_config.offset = offset;
                printf("Sensor offset: %d, Polarity: %d, ", offset, polarity);
            }
            printf("Voltage %d\n", input_voltage);
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("Now you can tune the offset clk: %d\n", motorcontrol_config.hall_offset[0]);
            else
                printf("Now you can tune the offset cclk: %d\n", motorcontrol_config.hall_offset[1]);
            printf("Enter c to start the auto tuning\nor enter a value to set the offset manually\n");
            break;
        //auto tune the offset by mesuring the current consumption
        case 'c':
            printf("Starting auto tuning...\n(This could take around 30 seconds)\n");
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING)) {
                motorcontrol_config.hall_offset[0] = auto_tuning_current(i_commutation, i_adc, input_voltage);
                printf("auto tuned offset clk: %d\n", motorcontrol_config.hall_offset[0]);
            } else {
                motorcontrol_config.hall_offset[1] = auto_tuning_current(i_commutation, i_adc, input_voltage);
                printf("auto tuned offset cclk: %d\n", motorcontrol_config.hall_offset[1]);
            }
            i_commutation.set_config(motorcontrol_config);
            break;
        //set voltage
        case 'v':
            input_voltage = value * sign;
            i_commutation.set_voltage(input_voltage);
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, offset clk: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, offset cclk: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        //reverse voltage
        case 'r':
            input_voltage = -input_voltage;
            i_commutation.set_voltage(input_voltage);
            if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
                printf("voltage: %i, offset clk: %d\n", input_voltage, motorcontrol_config.hall_offset[0]);
            else
                printf("voltage: %i, offset cclk: %d\n", input_voltage, motorcontrol_config.hall_offset[1]);
            break;
        //set sensor offset
        case 's':
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
        //print offsets, voltage and polarity
        case 'p':
            if (motorcontrol_config.commutation_sensor == AMS_SENSOR || motorcontrol_config.commutation_sensor == BISS_SENSOR)
                printf("Polarity %d, Sensor offset %d, ", polarity, offset);
            if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                printf("Voltage %d\noffset clk %d (for positive voltage)\noffset cclk %d (for negative voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            else
                printf("Voltage %d\noffset clk %d (for negative voltage)\noffset cclk %d (for positive voltage)\n", input_voltage, motorcontrol_config.hall_offset[0], motorcontrol_config.hall_offset[1]);
            break;
        //reverse sensor direction
        case 'd':
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
        //set offset
        default:
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
    if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
        start_offset = motorcontrol_config.hall_offset[0];
    else
        start_offset = motorcontrol_config.hall_offset[1];
    //starting peak current and offset
    int best_offset = (start_offset & 4095);
    int min_current = find_peak_current(i_adc, 1000, 200);
    int last_min_current;
    //search forward then backward
    for (int j=0; j<2; j++) {
        int offset = start_offset;
        do {
            last_min_current = min_current;
            for (int i=0; i<25; i++) {
                unsigned int pos_offset = (offset & 4095);
                //update offset
                if ((input_voltage >= 0 && motorcontrol_config.bldc_winding_type == STAR_WINDING) || (input_voltage <= 0 && motorcontrol_config.bldc_winding_type == DELTA_WINDING))
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
    return best_offset;
}

/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>


int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
{
    printf("\n\n\n\n\nsending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    delay_milliseconds(30000);

    int offset=i_motorcontrol.set_calib(0);
    printf("detected offset is: %i\n", offset);
    return offset;
}

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol)
{
    delay_milliseconds(500);
    printf(">>  ADVANCED FOC DEMO STARTING ...\n");

    printf(">>   applicable commands:\n");
    printf(" a => auto offset detection,           |    b  => enable/disable the breaks\n");
    printf(" t => enable/disable torque controller,|    ox => set offset to x \n");
    printf(" p => print the actual offset,         |    r  => reverse the torque\n");
    printf("                                       | Enter => set torque to 0\n");

    int torque_ref = 0;
    int brake_flag = 1;
    int torque_control_flag = 0;

    MotorcontrolConfig motorcontrol_config = i_motorcontrol.get_config();
    i_motorcontrol.set_break_status(1);


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
            auto_offset(i_motorcontrol);
            break;
        case 'b':
            if (brake_flag)
            {
                brake_flag = 0;
            }
            else
            {
                brake_flag = 1;
            }
            i_motorcontrol.set_break_status(brake_flag);
            break;

        //enable and disable torque controller
        case 't':
            if (torque_control_flag == 0) {
                torque_control_flag = 1;
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");
            } else {
                torque_control_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control deactivated\n");
            }
            break;

        //set offset
        case 'o':
            printf("set offset to %d\n", value);
            i_motorcontrol.set_offset_value(value);
            break;

        //print offsets, voltage and polarity
        case 'p':
            printf("offset %d\n", i_motorcontrol.set_calib(0));
            break;
        //reverse voltage
        case 'r':
            torque_ref = -torque_ref;
            i_motorcontrol.set_torque(torque_ref);
            break;

        //set torque to 0
        default:
            torque_ref = value * sign;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
            break;
        }
        delay_milliseconds(10);
    }
}

void demo_torque_control(interface MotorcontrolInterface client i_motorcontrol)
{
    int offset=0;

    printf(">>  DEMO TORQUE CONTROL STARTING ...\n");
    delay_milliseconds(4000);

    printf(">>  UNLOCKING THE BRAKE ...\n");
    i_motorcontrol.set_break_status(1);
    delay_milliseconds(2000);

    printf(">>  STARTING OFFSET DETECTION ...\n");
    auto_offset(i_motorcontrol);
    delay_milliseconds(30000);
    offset=i_motorcontrol.set_calib(0);
    printf("DETECTED OFFSET IS %d\n", offset);

    printf(">>  LOCKING THE BRAKE ...\n");
    i_motorcontrol.set_break_status(0);
    delay_milliseconds(2000);

    printf("set offset to %d\n", offset);
    i_motorcontrol.set_offset_value(offset);
    delay_milliseconds(2000);


    printf(">>  ENABLING THE CONTROL ...\n");
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(2000);



    printf(">>   applicable commands:\n");
    printf(" a => auto offset detection,           |    b  => enable/disable the breaks\n");
    printf(" t => enable/disable torque controller,|    ox => set offset to x \n");
    printf(" p => print the actual offset,         |    r  => reverse the torque\n");
    printf("                                       | Enter => set torque to 0\n");

    int torque_ref = 0;
    int brake_flag = 1;
    int torque_control_flag = 0;

    i_motorcontrol.set_break_status(1);


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
            auto_offset(i_motorcontrol);
            break;
        case 'b':
            if (brake_flag)
            {
                brake_flag = 0;
            }
            else
            {
                brake_flag = 1;
            }
            i_motorcontrol.set_break_status(brake_flag);
            break;

        //enable and disable torque controller
        case 't':
            if (torque_control_flag == 0) {
                torque_control_flag = 1;
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");
            } else {
                torque_control_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control deactivated\n");
            }
            break;

        //set offset
        case 'o':
            printf("set offset to %d\n", value);
            i_motorcontrol.set_offset_value(value);
            break;

        //print offsets, voltage and polarity
        case 'p':
            printf("offset %d\n", i_motorcontrol.set_calib(0));
            break;
        //reverse voltage
        case 'r':
            torque_ref = -torque_ref;
            i_motorcontrol.set_torque(torque_ref);
            break;

        //set torque to 0
        default:
            torque_ref = value * sign;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
            break;
        }
        delay_milliseconds(10);
    }
}


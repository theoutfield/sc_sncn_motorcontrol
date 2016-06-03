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
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    delay_milliseconds(30000);

    int offset=i_motorcontrol.set_calib(0);
    printf("Detected offset is: %i\n", offset);
    return offset;
}

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol, client interface TuningInterface ?i_tuning)
{
    delay_milliseconds(500);
    printf(">>  ADVANCED FOC DEMO STARTING ...\n");

    printf(">>   applicable commands:\n");
    printf(" a => auto offset detection,           |    b  => enable/disable the brake\n");
    printf(" t => enable/disable torque controller,|    ox => set offset to x \n");
    printf(" p => print the actual offset,         |    r  => reverse the torque\n");
    printf("                                       | Enter => set torque to 0\n");

    int torque_ref = 0;
    int brake_flag = 1;
    int torque_control_flag = 1;

    i_motorcontrol.set_brake_status(1);
    i_motorcontrol.set_torque_control_enabled();
    if (!isnull(i_tuning))
        i_tuning.set_limit(position_limit);


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
        //set brake
        case 'b':
            if (brake_flag) {
                brake_flag = 0;
                printf("Brake blocking\n");
            } else {
                brake_flag = 1;
                printf("Brake released\n");
            }
            i_motorcontrol.set_brake_status(brake_flag);
            break;

        //position limit
        case 'l':
            if (!isnull(i_tuning))
                i_tuning.set_limit(value * sign);
            break;

        //set offset
        case 'o':
            printf("set offset to %d\n", value);
            i_motorcontrol.set_offset_value(value);
            break;

        //print offset
        case 'p':
            printf("offset %d\n", i_motorcontrol.set_calib(0));
            break;
        //reverse voltage
        case 'r':
            torque_ref = -torque_ref;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
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

        //set torque
        default:
            torque_ref = value * sign;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
            break;
        }
        delay_milliseconds(10);
    }
}


void position_limiter(interface TuningInterface server i_tuning, client interface MotorcontrolInterface i_motorcontrol)
{
    timer t;
    unsigned ts;
    t :> ts;
    int position_limit = 0;
    int print_position_limit = 0;
    int count = 0;
    int velocity = 0;

    while(1) {
        select {
        case t when timerafter(ts) :> void:

            count = i_motorcontrol.get_position_actual();
            velocity = i_motorcontrol.get_velocity_actual();

            //postion limiter
            if (position_limit > 0) {
                if (count >= position_limit && velocity > 10) {
                    i_motorcontrol.set_torque(0);
                    if (print_position_limit >= 0) {
                        print_position_limit = -1;
                        printf("up limit reached\n");
                    }
                } else if (count <= -position_limit && velocity < -10) {
                    i_motorcontrol.set_torque(0);
                    if (print_position_limit <= 0) {
                        print_position_limit = 1;
                        printf("down limit reached\n");
                    }
                }
            }
            t :> ts;
            ts += USEC_STD * 1000;
            break;

        case i_tuning.set_limit(int in_limit):
            if (in_limit < 0) {
                position_limit = in_limit;
                printf("Position limit disabled\n");
            } else if (in_limit > 0) {
                printf("Position limited to %d ticks\n", in_limit);
                position_limit = in_limit;
            }
            break;

        }//end select
    }//end while
}//end function

void demo_torque_control(interface MotorcontrolInterface client i_motorcontrol)
{
    int offset=0;

    printf(">>  DEMO TORQUE CONTROL STARTING ...\n");
    delay_milliseconds(4000);

    printf(">>  UNLOCKING THE BRAKE ...\n");
    i_motorcontrol.set_brake_status(1);
    delay_milliseconds(2000);

    printf(">>  STARTING OFFSET DETECTION ...\n");
    auto_offset(i_motorcontrol);
    delay_milliseconds(30000);
    offset=i_motorcontrol.set_calib(0);
    printf("DETECTED OFFSET IS %d\n", offset);

    printf(">>  LOCKING THE BRAKE ...\n");
    i_motorcontrol.set_brake_status(0);
    delay_milliseconds(2000);

    printf("set offset to %d\n", offset);
    i_motorcontrol.set_offset_value(offset);
    delay_milliseconds(2000);


    printf(">>  ENABLING THE CONTROL ...\n");
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(2000);

}

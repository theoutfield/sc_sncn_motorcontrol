/*
 * tuning.xc

 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */

#include <stdio.h>
#include <ctype.h>

#include <tuning.h>

#include <position_feedback_service.h>
#include <motorcontrol_service.h>

#include <xscope.h>

int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
{
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    while(i_motorcontrol.set_calib(0)==-1) delay_milliseconds(50);//wait until offset is detected

    int offset=i_motorcontrol.set_calib(0);
    printf("Detected offset is: %i\n", offset);

    int proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();
    if(proper_sensor_polarity == 1) {
        printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
        i_motorcontrol.set_torque_control_enabled();
    } else {
        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
    }
    return offset;
}

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol, client interface TuningInterface ?i_tuning)
{

    int period_us;     // torque generation period in micro-seconds
    int pulse_counter; // number of generated pulses

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

            //play sound!
            case 's':
                for(period_us=400;period_us<=(1*1000);(period_us+=400))
                {
                    if(period_us<3000) period_us-=300;

                    for(pulse_counter=0;pulse_counter<=(50000/period_us);pulse_counter++)//total period = period * pulse_counter=1000000 us
                    {
                        i_motorcontrol.set_torque(80);
                        delay_microseconds(period_us);
                        i_motorcontrol.set_torque(-80);
                        delay_microseconds(period_us);
                    }
                }
                i_motorcontrol.set_torque(0);
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

/*
 * The following function shows how to work with torque controller.
 * It is able to:
 *  - independently lock and unlock the brakes
 *  - automatically find the offset
 *  - read or set the offset
 *  - independently enable and disable the control
 *  - send the reference value of torque to torque controller
 *  - show the variables on xscope
 *
 *  As a demo, the motor generates an oscilating torque with a frequency range between 10 Hz  and  3 kHz.
 */
void demo_torque_control(interface MotorcontrolInterface client i_motorcontrol)
{
    int period_us;     // torque generation period in micro-seconds
    int pulse_counter; // number of generated pulses


    int loop_counter=0;
    int proper_sensor_polarity=0;

    int torque_ref = 0;
    int brake_flag = 0;
    int torque_control_flag = 0;

    int offset=3600;

    UpstreamControlData upstream_control_data;

    MotorcontrolConfig motorcontrol_config;


    printf(">>  ADVANCED FOC DEMO STARTING ...\n");
    printf(">>   applicable commands:\n");
    printf(" a => auto offset detection,           |    b  => enable/disable the brake\n");
    printf(" t => enable/disable torque controller,|    ox => set offset to x \n");
    printf(" p => print the actual offset,         |    r  => reverse the torque\n");
    printf(" x => show on xscope for 20 seconds,   | Enter => set torque to 0\n");



    i_motorcontrol.set_offset_value(offset);
    printf("set offset to %d\n", i_motorcontrol.set_calib(0));

    i_motorcontrol.set_brake_status(1);
    i_motorcontrol.set_torque_control_enabled();

    fflush(stdout);
    //read and adjust the offset.
    while (1)
    {
        char mode = 0;
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n')
        {
            if(isdigit(c)>0)
            {
                value *= 10;
                value += c - '0';
            }
            else if (c == '-')
            {
                sign = -1;
            }
            else if (c != ' ')
                mode = c;
        }
        switch(mode)
        {
        //auto find offset
        case 'a':
            printf("Sending offset_detection command ...\n");
            i_motorcontrol.set_offset_detection_enabled();

            while(i_motorcontrol.set_calib(0)==-1) delay_milliseconds(50);//wait until offset is detected


            offset=i_motorcontrol.set_calib(0);
            printf("Detected offset is: %i\n", offset);

            printf("set offset to %d\n", offset);
            i_motorcontrol.set_offset_value(offset);
            delay_milliseconds(2000);

            proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();

            if(proper_sensor_polarity == 1)
            {
                printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
                i_motorcontrol.set_torque_control_enabled();
            }
            else
            {
                printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
            }
            break;

            //set brake
        case 'b':
            if (brake_flag)
            {
                brake_flag = 0;
                printf("Brake blocking\n");
            }
            else
            {
                brake_flag = 1;
                printf("Brake released\n");
            }
            i_motorcontrol.set_brake_status(brake_flag);
            break;

            //set offset
        case 'o':
            printf("set offset to %d\n", value);
            i_motorcontrol.set_offset_value(value);
            break;

        case 'p':
            motorcontrol_config = i_motorcontrol.get_config();
            printf("previous value: %d\n", motorcontrol_config.current_P_gain);

            motorcontrol_config.current_P_gain =  value;
            i_motorcontrol.set_torque_control_disabled();
            printf("Torque control disabled\n");
            delay_milliseconds(100);

            i_motorcontrol.set_config(motorcontrol_config);
            printf("new value: %d\n", motorcontrol_config.current_P_gain);
            delay_milliseconds(100);

            i_motorcontrol.set_torque_control_enabled();
            printf("Torque control enabled\n");
            delay_milliseconds(100);

            break;

            //print offset
        case 'i':
            motorcontrol_config = i_motorcontrol.get_config();
            printf("previous reversed_delay: %d\n", motorcontrol_config.current_I_gain);

            motorcontrol_config.current_I_gain =  value;

            i_motorcontrol.set_torque_control_disabled();
            printf("Torque control disabled\n");
            delay_milliseconds(100);

            i_motorcontrol.set_config(motorcontrol_config);
            printf("new reversed_delay: %d\n", motorcontrol_config.current_I_gain);
            delay_milliseconds(100);

            i_motorcontrol.set_torque_control_enabled();
            printf("Torque control enabled\n");
            delay_milliseconds(100);

            break;


            //reverse torque
        case 'r':
            torque_ref = -torque_ref;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
            break;

            //enable and disable torque controller
        case 't':
            if (torque_control_flag == 0)
            {
                torque_control_flag = 1;
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");
            }
            else
            {
                torque_control_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control deactivated\n");
            }
            break;

            //play sound!
        case 'm':
            torque_ref=value;
            for(period_us=400;period_us<=(5*400);(period_us+=400))
            {
                for(pulse_counter=0;pulse_counter<=(50000/period_us);pulse_counter++)//total period = period * pulse_counter=1000000 us
                {
                    i_motorcontrol.set_torque(torque_ref);
                    delay_microseconds(period_us);
                    i_motorcontrol.set_torque(-torque_ref);
                    delay_microseconds(period_us);
                }
            }


            for(period_us=(5*400);period_us>=400;(period_us-=400))
            {
                for(pulse_counter=0;pulse_counter<=(50000/period_us);pulse_counter++)//total period = period * pulse_counter=1000000 us
                {
                    i_motorcontrol.set_torque(torque_ref);
                    delay_microseconds(period_us);
                    i_motorcontrol.set_torque(-torque_ref);
                    delay_microseconds(period_us);
                }
            }

            i_motorcontrol.set_torque(0);
            break;

            //go to safe mode torque
        case 's':

            printf(">>  GO TO SAFE_TORQUE_OFF MODE IN TWO SECONDS ...\n");
            delay_milliseconds(2000);
            i_motorcontrol.set_safe_torque_off_enabled();

            break;

            //show on xscope for 10 seconds!
        case 'x':
            printf("activate xscope during 20 seconds ...\n");
            for(int i=0; i<=20000;i++)
            {
                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                xscope_int(COMPUTED_TORQUE, upstream_control_data.computed_torque);
                xscope_int(V_DC, upstream_control_data.V_dc);
                xscope_int(ANGLE, upstream_control_data.angle);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(TEMPERATURE, upstream_control_data.temperature);
                xscope_int(FAULT_CODE, upstream_control_data.error_status);

                delay_milliseconds(1);
            }
            break;

        case 'z':
            printf("reset faults, and check status ...\n");
            i_motorcontrol.reset_faults();

            delay_milliseconds(500);
            upstream_control_data = i_motorcontrol.update_upstream_control_data();

            if(upstream_control_data.error_status != NO_FAULT)
                printf(">>  FAULT ID %i DETECTED ...\n", upstream_control_data.error_status);

            if(upstream_control_data.error_status == NO_FAULT)
            {
                printf(">>  FAULT REMOVED ...\n");

                torque_control_flag = 1;
                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control activated\n");

                brake_flag = 1;
                i_motorcontrol.set_brake_status(brake_flag);
                printf("Brake released\n");

                printf("set offset to %d\n", offset);
                i_motorcontrol.set_offset_value(offset);


            }
            break;

            //set torque
        default:
            torque_ref = value * sign;
            i_motorcontrol.set_torque(torque_ref);
            printf("torque %d\n", torque_ref);
            break;
        }
    }

}




/*
 * tuning.xc

 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */

#include <stdio.h>
#include <ctype.h>

#include <user_interface_service.h>

#include <position_feedback_service.h>
#include <motor_control_interfaces.h>
#include <xscope.h>

/*
 * The following service shows how to directly work with module_torque_control.
 * It is able to:
 *  - automatically find motor offset
 *  - read/set motor offsett
 *  - enable/disable torque controller
 *  - send the reference value of the torque to motor_control_service
 *  - lock/unlock the brakes
 *
 * @param i_motorcontrol -> interface of type MotorControlInterface to communicate with torque controller
 */
void demo_torque_control(interface MotorControlInterface client i_motorcontrol)
{

    int torque_ref = 0;
    int brake_flag = 0;
    int torque_control_flag = 0;

    int offset=0;

    UpstreamControlData upstream_control_data;
    MotorcontrolConfig motorcontrol_config;

    printf(" DEMO_TORQUE_CONTROL started...\n");
    i_motorcontrol.set_brake_status(1);
    i_motorcontrol.set_torque_control_enabled();

    upstream_control_data = i_motorcontrol.update_upstream_control_data();

    fflush(stdout);
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
        //automatic offset detection
        case 'a':
                int proper_sensor_polarity=0;

                printf("automatic offset detection started ...\n");
                i_motorcontrol.set_offset_detection_enabled();

                while(i_motorcontrol.get_offset()==-1) delay_milliseconds(50);//wait until offset is detected

                proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();

                if(proper_sensor_polarity == 1)
                {
                    offset=i_motorcontrol.get_offset();
                    printf("detected offset is: %i\n", offset);

                    printf("offset is set to %d\n", offset);
                    i_motorcontrol.set_offset_value(offset);

                    motorcontrol_config = i_motorcontrol.get_config();
                    if(motorcontrol_config.commutation_sensor==HALL_SENSOR)
                    {
                        for (int i=0;i<6;i++)
                        {
                            printf("     hall_state_angle[%d]: %d\n", i, motorcontrol_config.hall_state[i]);
                        }
                    }

                    delay_milliseconds(2000);
                    i_motorcontrol.set_torque_control_enabled();
                }
                else
                {
                    printf(">>  ERROR: wrong polarity for commutation position sensor\n");
                }
                break;

        //enable/disable brake
        case 'b':
                if (brake_flag)
                {
                    brake_flag = 0;
                    printf("brake enabled\n");
                }
                else
                {
                    brake_flag = 1;
                    printf("brake disabled\n");
                }
                i_motorcontrol.set_brake_status(brake_flag);
                break;

        //set offset
        case 'o':
                offset = value;
                printf("offset set to %d\n", offset);
                i_motorcontrol.set_offset_value(offset);
                break;

        //set kp for torque controller
        case 'p':
                motorcontrol_config = i_motorcontrol.get_config();

                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control disabled\n");
                delay_milliseconds(100);

                motorcontrol_config.torque_P_gain =  value;
                i_motorcontrol.set_config(motorcontrol_config);
                printf("set kp to %d\n", motorcontrol_config.torque_P_gain);
                delay_milliseconds(100);

                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control enabled\n");
                delay_milliseconds(100);
                break;

        //set ki for torque controller
        case 'i':
                motorcontrol_config = i_motorcontrol.get_config();

                i_motorcontrol.set_torque_control_disabled();
                printf("Torque control disabled\n");
                delay_milliseconds(100);

                motorcontrol_config.torque_I_gain =  value;
                i_motorcontrol.set_config(motorcontrol_config);
                printf("set ki to %d\n", motorcontrol_config.torque_I_gain);
                delay_milliseconds(100);

                i_motorcontrol.set_torque_control_enabled();
                printf("Torque control enabled\n");
                delay_milliseconds(100);
                break;

        //reverse the direction of reference torque
        case 'r':
                torque_ref = -torque_ref;
                i_motorcontrol.set_torque(torque_ref);
                printf("torque %d [milli-Nm]\n", torque_ref);
                break;

        //enable/disable torque controller
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

        //play music! it sends a square-wave reference signal (with audible frequency) to torque controller.
        case 'm':
                int period_us;     // torque period in micro-seconds
                int pulse_counter; // number of generated pulses
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



                        i_motorcontrol.set_torque((-torque_ref*110)/100);
                        delay_microseconds((5*period_us)/100);

                        i_motorcontrol.set_torque(-torque_ref);
                        delay_microseconds(period_us);
                    }
                }

                i_motorcontrol.set_torque(0);
                break;

        //safe mode torque (all inverter power swieches open)
        case 's':
                printf("safe torque off mode started\n");
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

            delay_milliseconds(1);
            motorcontrol_config = i_motorcontrol.get_config();
            if(torque_ref>motorcontrol_config.max_torque || torque_ref<-motorcontrol_config.max_torque)
            {
                upstream_control_data = i_motorcontrol.update_upstream_control_data();
                printf("above limits! torque %d [milli-Nm]\n", upstream_control_data.torque_set);
            }
            else
                printf("torque %d [milli-Nm]\n", torque_ref);
            break;
        }
    }

}




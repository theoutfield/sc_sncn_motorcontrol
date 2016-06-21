/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

/**
 * @file app_test_contelec_rotary_sensor.xc
 * @brief Test illustrates usage of the CONTELEC rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

/***** Sensor Test *****/
#include <position_feedback_service.h>
#include <ctype.h>
#include <stdio.h>

/***** Motor Test *****/
#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <torque_control.h>

/*********** Sensor Test ***********/
PositionFeedbackPorts position_feedback_ports = SOMANET_IFM_POSITION_FEEDBACK_PORTS;

/*********** Motor Test ***********/
PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;


/* Test CONTELEC Sensor Client */
void contelec_encoder_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;
    int status = 0;
    timer t;
    unsigned start_time, end_time;

    while(1) {
        /* get position from CONTELEC Sensor */
        t :> start_time;
        {void, void, status} = i_position_feedback.get_real_position();
        t :> end_time;
        {count, position} = i_position_feedback.get_position();

        /* get angle and velocity from CONTELEC Sensor */
//        velocity = i_position_feedback.get_velocity();

        //electrical_angle = i_position_feedback.get_angle();

        if (!isnull(i_shared_memory)) {
            { electrical_angle, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }


        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);
        xscope_int(STATUS, status*1000);
//        xscope_int(TIME, status);

        delay_microseconds(10);
    }
}

//finding the offset function
int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
{
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    int offset = -1;
    while (offset == -1) {
        delay_milliseconds(50);//wait until offset is detected
        offset = i_motorcontrol.set_calib(0);
    }

    printf("Detected offset is: %i\n", offset);
//    printf(">>  CHECK PROPER OFFSET POLARITY ...\n");
    int proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();
    if(proper_sensor_polarity == 1) {
        printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
        i_motorcontrol.set_torque_control_enabled();
    } else {
        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
        printf(">>  Please either change CONTELEC_POLARITY parameter [0 - normal, 1 - inverted] or flip any two motor phases\n");
    }
    return offset;
}

void contelec_encoder_commands_test(client interface PositionFeedbackInterface i_position_feedback, interface MotorcontrolInterface client i_motorcontrol) {
    char status;
    int multiturn;
//    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned start_time, end_time;
    timer t;
    int offset = -1;

    delay_milliseconds(500);
    PositionFeedbackConfig position_feedback_config = i_position_feedback.get_config();
    printstr(">>   SOMANET CONTELEC SENSOR COMMANDS SERVICE STARTING...\n");

    while(1) {
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
        //set angle
        case 'a':
            printf("Set angle to %d\nnew offset %d\n", value, i_position_feedback.set_angle(value));
            break;
        //auto offset tuning
        case 'b':
            offset = auto_offset(i_motorcontrol);
            printf("using commutation offset to %d\n", offset);
            break;
        //set calibration point
        case 'c':
            i_position_feedback.send_command(CONTELEC_CALIB_TBL_POINT, value, 16);
            printf("set calibration point %d\n", value);
            break;
        //change direction
        case 'd':
            i_position_feedback.send_command(CONTELEC_CONF_DIR, value, 8);
            printf("direction %d\n", value);
            break;
        //filter
        case 'f':
            i_position_feedback.send_command(CONTELEC_CONF_FILTER, value, 8);
            printf("filter %d\n", value);
            break;
        //set offset
        case 'o':
            position_feedback_config = i_position_feedback.get_config();
            position_feedback_config.contelec_config.offset = value * sign;
            i_position_feedback.set_config(position_feedback_config);
            printf("offset %d\n", value * sign);
            break;
        //set position
        case 'p':
            i_position_feedback.set_position(value*sign);
            break;
        //set multiturn
        case 'm':
            i_position_feedback.send_command(CONTELEC_CONF_MTPRESET, value*sign, 16);
            printf("multiturn\n");
            break;
        //set toruqe
        case 'q':
            if(offset != -1){
                int torque = value*sign;
                i_motorcontrol.set_torque(torque);
                printf("Torque %d\n", torque);
            }
            else {
                printf("Ofset is not found or invalid, please type command 'b' first \n");
            }
            break;
        //reset sensor
        case 'r':
            i_position_feedback.send_command(CONTELEC_CTRL_RESET, 0, 0);
            printf("reset\n");
            break;
        //set singleturn
        case 's':
            i_position_feedback.send_command(CONTELEC_CONF_STPRESET, value, 16);
            printf("singleturn\n");
            break;
        //calibration table size
        case 't':
            i_position_feedback.send_command(CONTELEC_CALIB_TBL_SIZE, value, 16);
            printf("calibration table size %d\n", value);
            break;
        //save
        case 'v':
            i_position_feedback.send_command(CONTELEC_CTRL_SAVE, 0, 0);
            i_position_feedback.send_command(CONTELEC_CTRL_RESET, 0, 0);
            printf("save\n");
            break;
        //set zero position
        case 'z':
            i_position_feedback.send_command(CONTELEC_CONF_NULL, 0, 0);
            printf("zero\n");
            break;
        //print the count and the time to get it
        default:
            t :> start_time;
            {multiturn, singleturn_raw, status} = i_position_feedback.get_real_position();
            t :> end_time;
            printf("time %d, count %d\n", (end_time-start_time)/USEC_STD, multiturn);
            break;
        }
        delay_milliseconds(10);
    }
}


int main(void)
{
    /*********** Sensor Test ***********/
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];
    /*********** Motor Test ***********/
    interface WatchdogInterface i_watchdog[2];
    interface update_pwm i_update_pwm;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[4];


    par
    {
        on tile[APP_TILE]: contelec_encoder_commands_test(i_position_feedback[1], i_motorcontrol[0]);

        on tile[IFM_TILE]: par {
//            contelec_encoder_test(i_position_feedback[0], i_shared_memory[1]);

            /*********** Motor Test ***********/

            {
                /* Watchdog Service */
                delay_milliseconds(500);
                watchdog_service(wd_ports,i_watchdog);
            }

            {
                pwm_config(pwm_ports);

                delay_milliseconds(1000);
                pwm_service_task(_MOTOR_ID, pwm_ports, i_update_pwm, DUTY_START_BRAKE, DUTY_MAINTAIN_BRAKE);
            }

            /* ADC Service */
            {
                delay_milliseconds(1500);
                adc_service(adc_ports, null/*c_trigger*/, i_adc /*ADCInterface*/, i_watchdog[1]);
            }


            /* Motor Control Service */
            {
                delay_milliseconds(2000);

                MotorcontrolConfig motorcontrol_config;

                motorcontrol_config.v_dc =  VDC;
                motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
                motorcontrol_config.commutation_angle_offset=COMMUTATION_OFFSET_CLK;
                motorcontrol_config.polarity_type=MOTOR_POLARITY;

                motorcontrol_config.current_P_gain =  TORQUE_Kp;

                motorcontrol_config.pole_pair =  POLE_PAIRS;
                motorcontrol_config.max_torque =  MAXIMUM_TORQUE;
                motorcontrol_config.phase_resistance =  PHASE_RESISTANCE;
                motorcontrol_config.phase_inductance =  PHASE_INDUCTANCE;

                motorcontrol_config.protection_limit_over_current =  I_MAX;
                motorcontrol_config.protection_limit_over_voltage =  V_DC_MAX;
                motorcontrol_config.protection_limit_under_voltage = V_DC_MIN;

                Motor_Control_Service( fet_driver_ports, motorcontrol_config, i_adc[0],
                        i_shared_memory[1],
                        i_watchdog[0], i_motorcontrol, i_update_pwm);
            }

            /*********** Sensor Test ***********/

            /* Shared memory Service */
            memory_manager(i_shared_memory, 2);


            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = CONTELEC_SENSOR;
                position_feedback_config.contelec_config.filter = CONTELEC_FILTER;
                position_feedback_config.contelec_config.polarity = CONTELEC_POLARITY;
                position_feedback_config.contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                position_feedback_config.contelec_config.offset = CONTELEC_OFFSET;
                position_feedback_config.contelec_config.pole_pairs = 5;
                position_feedback_config.contelec_config.timeout = CONTELEC_TIMEOUT;
                position_feedback_config.contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                position_feedback_config.contelec_config.enable_push_service = PushAll;

                position_feedback_service(position_feedback_ports, position_feedback_config, i_shared_memory[0], i_position_feedback, null, null, null, null);
            }

        }
    }

    return 0;
}

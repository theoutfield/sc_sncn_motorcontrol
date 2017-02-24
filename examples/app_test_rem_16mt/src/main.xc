/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


/**
 * @file app_test_rem_16mt_rotary_sensor.xc
 * @brief Test illustrates usage of the REM_16MT rotary sensor to get position, velocity, and electrical angle information
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
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
/*********** Sensor Test ***********/
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

/*********** Motor Test ***********/
PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;


/* Test REM_16MT Sensor Client */
void rem_16mt_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int angle = 0;
    int status = 0;
    timer t;
    unsigned start_time, end_time;

    while(1) {
        /* get position from REM_16MT Sensor */
        t :> start_time;
        {count, position, status } = i_position_feedback.get_position();
        t :> end_time;

        /* get angle and velocity from REM_16MT Sensor */
        //        velocity = i_position_feedback.get_velocity();

        //electrical_angle = i_position_feedback.get_angle();

        if (!isnull(i_shared_memory)) {
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            angle = upstream_control_data.angle;
            count = upstream_control_data.position;
            velocity = upstream_control_data.velocity;
        }


        //        xscope_int(COUNT, count);
        //        xscope_int(POSITION, position);
        //        xscope_int(ANGLE, angle);
        //        xscope_int(VELOCITY, velocity);
        //        xscope_int(STATUS, status*1000);
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
        printf(">>  Please either change SENSOR_POLARITY parameter [0 - normal, 1 - inverted] or flip any two motor phases\n");
    }
    return offset;
}

void rem_16mt_commands_test(client interface PositionFeedbackInterface i_position_feedback, interface MotorcontrolInterface client i_motorcontrol) {
    char status;
    int multiturn;
    //    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned start_time, end_time;
    timer t;
    int offset = 0;

    delay_milliseconds(500);
    PositionFeedbackConfig position_feedback_config = i_position_feedback.get_config();
    printstr(">>   SOMANET REM_16MT SENSOR COMMANDS SERVICE STARTING...\n");
    i_motorcontrol.set_torque_control_enabled();
    i_motorcontrol.set_brake_status(1);

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
            //auto offset tuning
        case 'b':
            offset = auto_offset(i_motorcontrol);
            printf("using commutation offset to %d\n", offset);
            break;
            //set calibration point
        case 'c':
            i_position_feedback.send_command(REM_16MT_CALIB_TBL_POINT, value, 16);
            printf("set calibration point %d\n", value);
            break;
            //change direction
        case 'd':
            i_position_feedback.send_command(REM_16MT_CONF_DIR, value, 8);
            printf("direction %d\n", value);
            break;
            //filter
        case 'f':
            i_position_feedback.send_command(REM_16MT_CONF_FILTER, value, 8);
            printf("filter %d\n", value);
            break;
            //set offset
        case 'o':
            position_feedback_config = i_position_feedback.get_config();
            position_feedback_config.offset = value * sign;
            i_position_feedback.set_config(position_feedback_config);
            printf("offset %d\n", value * sign);
            break;
            //set position
        case 'p':
            i_position_feedback.set_position(value*sign);
            break;
            //set multiturn
        case 'm':
            i_position_feedback.send_command(REM_16MT_CONF_MTPRESET, value*sign, 16);
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
            i_position_feedback.send_command(REM_16MT_CTRL_RESET, 0, 0);
            printf("reset\n");
            break;
            //set singleturn
        case 's':
            i_position_feedback.send_command(REM_16MT_CONF_STPRESET, value, 16);
            printf("singleturn\n");
            break;
            //calibration table size
        case 't':
            i_position_feedback.send_command(REM_16MT_CALIB_TBL_SIZE, value, 16);
            printf("calibration table size %d\n", value);
            break;
            //save
        case 'v':
            i_position_feedback.send_command(REM_16MT_CTRL_SAVE, 0, 0);
            i_position_feedback.send_command(REM_16MT_CTRL_RESET, 0, 0);
            printf("save\n");
            break;
            //set zero position
        case 'z':
            i_position_feedback.send_command(REM_16MT_CONF_NULL, 0, 0);
            printf("zero\n");
            break;
            //set velocity loop time
        case 'l':
            position_feedback_config = i_position_feedback.get_config();
            position_feedback_config.velocity_compute_period = value;
            i_position_feedback.set_config(position_feedback_config);
            printf("velocity loop time %dus\n", position_feedback_config.velocity_compute_period);
            break;
            //print the count and the time to get it
        default:
            t :> start_time;
            {multiturn, singleturn_raw, status} = i_position_feedback.get_position();
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
    interface update_brake i_update_brake;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[2];


    par
    {
        on tile[APP_TILE]: rem_16mt_commands_test(i_position_feedback[1], i_motorcontrol[1]);

        on tile[IFM_TILE]: par
        {
            /* PWM Service */
            {
                pwm_config(pwm_ports);

                if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                    predriver(fet_driver_ports);

                //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                        i_update_brake, IFM_TILE_USEC);

            }

            /* ADC Service */
            {
                adc_service(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, SINGLE_ENDED);
            }

            /* Watchdog Service */
            {
                watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
            }

            /* Motor Control Service */
            {

                MotorcontrolConfig motorcontrol_config;

                motorcontrol_config.licence =  ADVANCED_MOTOR_CONTROL_LICENCE;
                motorcontrol_config.v_dc =  VDC;
                motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
                motorcontrol_config.polarity_type=MOTOR_POLARITY;
                motorcontrol_config.current_P_gain =  TORQUE_Kp;
                motorcontrol_config.current_I_gain =  TORQUE_Ki;
                motorcontrol_config.current_D_gain =  TORQUE_Kd;
                motorcontrol_config.pole_pair =  POLE_PAIRS;
                motorcontrol_config.commutation_sensor=SENSOR_1_TYPE;
                motorcontrol_config.commutation_angle_offset=COMMUTATION_OFFSET_CLK;
                motorcontrol_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                motorcontrol_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                motorcontrol_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                motorcontrol_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                motorcontrol_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                motorcontrol_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;
                motorcontrol_config.max_torque =  MAXIMUM_TORQUE;
                motorcontrol_config.phase_resistance =  PHASE_RESISTANCE;
                motorcontrol_config.phase_inductance =  PHASE_INDUCTANCE;
                motorcontrol_config.torque_constant =  PERCENT_TORQUE_CONSTANT;
                motorcontrol_config.current_ratio =  CURRENT_RATIO;
                motorcontrol_config.voltage_ratio =  VOLTAGE_RATIO;
                motorcontrol_config.rated_current =  RATED_CURRENT;
                motorcontrol_config.rated_torque  =  RATED_TORQUE;
                motorcontrol_config.percent_offset_torque =  PERCENT_OFFSET_TORQUE;
                motorcontrol_config.recuperation = RECUPERATION;
                motorcontrol_config.battery_e_max = BATTERY_E_MAX;
                motorcontrol_config.battery_e_min = BATTERY_E_MIN;
                motorcontrol_config.regen_p_max = REGEN_P_MAX;
                motorcontrol_config.regen_p_min = REGEN_P_MIN;
                motorcontrol_config.regen_speed_max = REGEN_SPEED_MAX;
                motorcontrol_config.regen_speed_min = REGEN_SPEED_MIN;
                motorcontrol_config.protection_limit_over_current =  I_MAX;
                motorcontrol_config.protection_limit_over_voltage =  V_DC_MAX;
                motorcontrol_config.protection_limit_under_voltage = V_DC_MIN;

                motor_control_service(motorcontrol_config, i_adc[0], i_shared_memory[1],
                        i_watchdog[0], i_motorcontrol, i_update_pwm, IFM_TILE_USEC);
            }

            /* Shared memory Service */
            [[distribute]] memory_manager(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = REM_16MT_SENSOR;
                position_feedback_config.resolution  = REM_16MT_SENSOR_RESOLUTION;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.velocity_compute_period = REM_16MT_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.pole_pairs  = POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = 0;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                position_feedback_service(null, null, null, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

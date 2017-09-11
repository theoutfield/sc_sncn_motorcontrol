/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file main.xc
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

/*********** Sensor Test ***********/
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

/*********** Motor Test ***********/
PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

void rem_16mt_commands_test(client interface PositionFeedbackInterface i_position_feedback, interface TorqueControlInterface client i_torque_control) {
    char status;
    int multiturn;
    unsigned int singleturn_filtered;
    unsigned start_time, end_time;
    timer t;
    int offset = 0;

    delay_milliseconds(500);
    PositionFeedbackConfig position_feedback_config = i_position_feedback.get_config();
    printstr(">>   SOMANET REM_16MT SENSOR COMMANDS SERVICE STARTING...\n");

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
        case 'a':
            printf("Sending offset_detection command ...\n");
            i_torque_control.set_offset_detection_enabled();
            while(i_torque_control.get_offset()==-1) delay_milliseconds(50);//wait until offset is detected

            if(i_torque_control.get_sensor_polarity_state() != 1) {
                printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
                offset = -1;
            } else{
                offset = i_torque_control.get_offset();
                printf("Detected offset is: %i\n", offset);

                //set offset to motorcontrol
                MotorcontrolConfig motorcontrol_config = i_torque_control.get_config();
                motorcontrol_config.commutation_angle_offset = offset;
                i_torque_control.set_config(motorcontrol_config);

                //start motorcontrol
                delay_milliseconds(500);
                i_torque_control.set_torque_control_enabled();
                i_torque_control.set_brake_status(1);
            }
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
        //set torque
        case 'q':
            if(offset != -1){
                int torque = value*sign;
                i_torque_control.set_torque(torque);
                printf("Torque %d\n", torque);
            } else {
                printf("Offset is not found or invalid, please type command 'a' first \n");
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
        //start motorcontrol
        case 'S':
            i_torque_control.set_torque_control_enabled();
            i_torque_control.set_brake_status(1);
            break;
        //calibration table size
        case 't':
            i_position_feedback.send_command(REM_16MT_CALIB_TBL_SIZE, value, 16);
            printf("calibration table size %d\n", value);
            break;
        //set calibration point
        case 'c':
            i_position_feedback.send_command(REM_16MT_CALIB_TBL_POINT, value, 16);
            printf("set calibration point %d\n", value);
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
        //set velocity compute period
        case 'l':
            position_feedback_config = i_position_feedback.get_config();
            position_feedback_config.velocity_compute_period = value;
            i_position_feedback.set_config(position_feedback_config);
            printf("velocity loop time %dus\n", position_feedback_config.velocity_compute_period);
            break;
        //print the position and the time to get it
        default:
            t :> start_time;
            {multiturn, singleturn_filtered, status} = i_position_feedback.get_position();
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
    interface shared_memory_interface i_shared_memory[3];
    /*********** Motor Test ***********/
    interface WatchdogInterface i_watchdog[2];
    interface UpdatePWMGeneral i_update_pwm;
    interface UpdateBrake i_update_brake;
    interface ADCInterface i_adc[2];
    interface TorqueControlInterface i_torque_control[2];


    par
    {
        on tile[APP_TILE]: rem_16mt_commands_test(i_position_feedback[1], i_torque_control[1]);

        on tile[IFM_TILE]: par
        {
            /* PWM Service */
            {
                pwm_config_general(pwm_ports);

                if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                    predriver(fet_driver_ports);

                pwm_service_general(pwm_ports, i_update_pwm, 15);
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

                motorcontrol_config.dc_bus_voltage =  DC_BUS_VOLTAGE;
                motorcontrol_config.phases_inverted = MOTOR_PHASES_CONFIGURATION;
                motorcontrol_config.torque_P_gain =  TORQUE_Kp;
                motorcontrol_config.torque_I_gain =  TORQUE_Ki;
                motorcontrol_config.torque_D_gain =  TORQUE_Kd;
                motorcontrol_config.pole_pairs =  MOTOR_POLE_PAIRS;
                motorcontrol_config.commutation_sensor=REM_16MT_SENSOR;
                motorcontrol_config.commutation_angle_offset=COMMUTATION_ANGLE_OFFSET;
                motorcontrol_config.max_torque =  MOTOR_MAXIMUM_TORQUE;
                motorcontrol_config.phase_resistance =  MOTOR_PHASE_RESISTANCE;
                motorcontrol_config.phase_inductance =  MOTOR_PHASE_INDUCTANCE;
                motorcontrol_config.torque_constant =  MOTOR_TORQUE_CONSTANT;
                motorcontrol_config.current_ratio =  CURRENT_RATIO;
                motorcontrol_config.voltage_ratio =  VOLTAGE_RATIO;
                motorcontrol_config.temperature_ratio =  TEMPERATURE_RATIO;
                motorcontrol_config.rated_current =  MOTOR_RATED_CURRENT;
                motorcontrol_config.rated_torque  =  MOTOR_RATED_TORQUE;
                motorcontrol_config.percent_offset_torque =  APPLIED_TUNING_TORQUE_PERCENT;
                motorcontrol_config.protection_limit_over_current =  PROTECTION_MAXIMUM_CURRENT;
                motorcontrol_config.protection_limit_over_voltage =  PROTECTION_MAXIMUM_VOLTAGE;
                motorcontrol_config.protection_limit_under_voltage = PROTECTION_MINIMUM_VOLTAGE;
                motorcontrol_config.protection_limit_over_temperature = TEMP_BOARD_MAX;
                for (int i = 0; i < 1024; i++)
                {
                    motorcontrol_config.torque_offset[i] = 0;
                }

                torque_control_service(motorcontrol_config, i_adc[0], i_shared_memory[2],
                        i_watchdog[0], i_torque_control, i_update_pwm, IFM_TILE_USEC, /*gpio_port_0*/null);
            }

            /* Shared memory Service */
            [[distribute]] shared_memory_service(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = REM_16MT_SENSOR;
                position_feedback_config.resolution  = REM_16MT_SENSOR_RESOLUTION;
                position_feedback_config.polarity    = SENSOR_POLARITY_NORMAL;
                position_feedback_config.velocity_compute_period = REM_16MT_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = HOME_OFFSET;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                position_feedback_config.gpio_config[0] = GPIO_OFF;
                position_feedback_config.gpio_config[1] = GPIO_OFF;
                position_feedback_config.gpio_config[2] = GPIO_OFF;
                position_feedback_config.gpio_config[3] = GPIO_OFF;

                position_feedback_service(null, null, null, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

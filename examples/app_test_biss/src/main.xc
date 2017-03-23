/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>



/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//BiSS libs
#include <watchdog_service.h>
#include <position_feedback_service.h>
#include <refclk.h>
#include <motor_control_interfaces.h>
#include <pwm_server.h>
#include <user_config.h>

/* Test BiSS Encoder Client */
void biss_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory) {
    timer t;
    unsigned int start_time, end_time;
    int count = 0;
    int real_count = 0;
    unsigned int angle = 0;
    int velocity = 0;
    unsigned int position = 0;
    unsigned int status = 0;

    while(1) {

        /* get position from BiSS Encoder */
        { count, position, status } = i_position_feedback.get_position();

        t :> start_time;
        /* get angle and velocity from BiSS Encoder */
        angle = i_position_feedback.get_angle();
        velocity = i_position_feedback.get_velocity();
        t :> end_time;


        if (!isnull(i_shared_memory)) {
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            angle = upstream_control_data.angle;
            count = upstream_control_data.position;
            velocity = upstream_control_data.velocity;
        }

        xscope_int(COUNT, count);                           //absolute count
        xscope_int(REAL_COUNT, real_count);                 //real internal absolute count
        xscope_int(POSITION, position);                     //singleturn position
        xscope_int(ANGLE, angle);                           //electrical angle
        xscope_int(VELOCITY, velocity);                     //velocity in rpm
        xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds
        xscope_int(STATUS, status);                         //error status


        switch(status) {
        case SENSOR_CHECKSUM_ERROR:
            xscope_int(CRC_ERROR, 1000);    //CRC error
            xscope_int(ERROR_BIT, 0);       //error bit
            xscope_int(WARNING_BIT, 0);     //warning bit
            break;
        case SENSOR_BISS_ERROR_BIT_ERROR:
            xscope_int(CRC_ERROR, 0);       //CRC error
            xscope_int(ERROR_BIT, 1000);    //error bit
            xscope_int(WARNING_BIT, 0);     //warning bit
            break;
        case SENSOR_BISS_WARNING_BIT_ERROR:
            xscope_int(CRC_ERROR, 0);       //CRC error
            xscope_int(ERROR_BIT, 0);       //error bit
            xscope_int(WARNING_BIT, 1000);  //warning bit
            break;
        case SENSOR_BISS_ERROR_AND_WARNING_BIT_ERROR:
            xscope_int(CRC_ERROR, 0);       //CRC error
            xscope_int(ERROR_BIT, 1000);    //error bit
            xscope_int(WARNING_BIT, 1000);  //warning bit
            break;
        default:
            xscope_int(CRC_ERROR, 0);       //CRC error
            xscope_int(ERROR_BIT, 0);       //error bit
            xscope_int(WARNING_BIT, 0);     //warning bit
            break;
        }

        delay_milliseconds(1);
    }
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

int main() {
    interface WatchdogInterface i_watchdog[2];
    interface shared_memory_interface i_shared_memory[2];
    interface PositionFeedbackInterface i_position_feedback[3];
    interface update_pwm i_update_pwm;
    interface update_brake i_update_brake;

    par {
        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: par {
            /* Test BiSS Encoder Client */
            biss_test(i_position_feedback[0], null);

            /* PWM Service */
            {
                pwm_config(pwm_ports);

                if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                    predriver(fet_driver_ports);

                //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                        i_update_brake, IFM_TILE_USEC);

            }

            /* Watchdog Service */
            {
                watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
            }

            /* Shared memory Service */
            [[distribute]] shared_memory_service(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = BISS_SENSOR;
                position_feedback_config.resolution  = BISS_SENSOR_RESOLUTION;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.velocity_compute_period = BISS_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = 0;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config.biss_config.busy = BISS_BUSY;
                position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, null, null, null, null,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }
    return 0;
}

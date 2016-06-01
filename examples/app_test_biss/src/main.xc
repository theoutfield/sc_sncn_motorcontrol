/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
//#include <CORE_BOARD_REQUIRED>
//#include <IFM_BOARD_REQUIRED>
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//BiSS libs
#include <watchdog_service.h>
#include <pwm_service.h>
#include <position_feedback_service.h>
#include <refclk.h>

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
        { count, position } = i_position_feedback.get_position();
        { real_count, void, status } = i_position_feedback.get_real_position();

        t :> start_time;
        /* get angle and velocity from BiSS Encoder */
        angle = i_position_feedback.get_angle();
        velocity = i_position_feedback.get_velocity();
//        { count, velocity, position, angle, status } = i_position_feedback.get_all();
        t :> end_time;


        if (!isnull(i_shared_memory)) {
            { angle, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }

        xscope_int(COUNT, count);                           //absolute count
        xscope_int(REAL_COUNT, real_count);                 //real internal absolute count
        xscope_int(POSITION, position);                     //singleturn position
        xscope_int(ANGLE, angle);                     //singleturn position
        xscope_int(VELOCITY, velocity);                     //velocity in rpm
        xscope_int(ERROR_BIT, (status&0b10) * 500);         //error bit, should be 0
        xscope_int(WARNING_BIT, (status&0b01) * 1000);      //warning bit, should be 0
        xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds
        xscope_int(TIME_INTERNAL, status);   //time to get the data in microseconds

        delay_milliseconds(1);
    }
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
//BISSPorts biss_ports = SOMANET_IFM_BISS_PORTS;
//PositionFeedbackPorts position_feedback_ports = { QEI_PORT, QEI_PORT_INPUT_MODE_SELECTION,SOMANET_IFM_GPIO_D0,
//        {IFM_TILE_CLOCK_2, IFM_TILE_CLOCK_3, SOMANET_IFM_GPIO_D3,SOMANET_IFM_GPIO_D1,SOMANET_IFM_GPIO_D2 } };
PositionFeedbackPorts position_feedback_ports = SOMANET_IFM_POSITION_FEEDBACK_PORTS;

int main() {
    chan c_pwm_ctrl; // pwm channels
    interface WatchdogInterface i_watchdog[2];
//    interface BISSInterface i_biss[5]; //array of interfaces for biss server
    interface BrakeInterface i_brake;
    interface shared_memory_interface i_shared_memory[2];
    interface PositionFeedbackInterface i_position_feedback[3];

    par {
        /* Test BiSS Encoder Client */
        on tile[APP_TILE]: biss_test(i_position_feedback[0], null);


        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: par {
            /* PWM Service */
            pwm_service( pwm_ports, c_pwm_ctrl, i_brake);
            i_brake.set_brake(0);

            /* Watchdog Service */
            watchdog_service(wd_ports, i_watchdog);

            // enable watchdog
            {
                delay_milliseconds(1);
                i_watchdog[0].start();
            }

            /* Shared memory Service */
            memory_manager(i_shared_memory, 2);

//            /* BiSS server */
//            {
//                BISSConfig biss_config;
//                biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
//                biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
//                biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
//                biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
//                biss_config.status_length = BISS_STATUS_LENGTH;
//                biss_config.crc_poly = BISS_CRC_POLY;
//                biss_config.pole_pairs = 2;
//                biss_config.polarity = BISS_POLARITY;
//                biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
//                biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
//                biss_config.timeout = BISS_TIMEOUT;
//                biss_config.max_ticks = BISS_MAX_TICKS;
//                biss_config.velocity_loop = BISS_VELOCITY_LOOP;
//                biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;
//                biss_config.enable_push_service = PushAll;
//
//                biss_service(biss_ports, biss_config, i_shared_memory[0], i_biss);
//            }

            /* Position service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = BISS_SENSOR;
                position_feedback_config.biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config.biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                position_feedback_config.biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                position_feedback_config.biss_config.status_length = BISS_STATUS_LENGTH;
                position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config.biss_config.pole_pairs = 2;
                position_feedback_config.biss_config.polarity = BISS_POLARITY;
                position_feedback_config.biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                position_feedback_config.biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config.biss_config.max_ticks = BISS_MAX_TICKS;
                position_feedback_config.biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                position_feedback_config.biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;
                position_feedback_config.biss_config.enable_push_service = PushAll;


                position_feedback_service(position_feedback_ports, position_feedback_config, i_shared_memory[0], i_position_feedback, null, null, null, null);
            }
        }
    }
    return 0;
}

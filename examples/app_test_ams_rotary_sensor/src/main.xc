/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <position_feedback_service.h>
#include <ctype.h>


SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;

/* Test AMS Sensor Client */
void ams_rotary_sensor_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;
    timer t;
    unsigned int start_time, end_time, time;


    while(1) {
        /* get position from AMS Sensor */
        {count, position} = i_position_feedback.get_position();

        /* get angle from AMS Sensor */
        electrical_angle = i_position_feedback.get_angle();

        /* get velocity from AMS Sensor */
        velocity = i_position_feedback.get_velocity();

        t :> start_time;
//        { count, velocity, position, electrical_angle, time } = i_position_feedback.get_ams_all();
        if (!isnull(i_shared_memory)) {
            { electrical_angle, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }
        t :> end_time;

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);
        xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds
        xscope_int(TIME_INTERNAL, time);   //time to get the data in microseconds

        delay_milliseconds(1);
    }
}


int main(void)
{
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        on tile[APP_TILE]: ams_rotary_sensor_test(i_position_feedback[0], null);

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: par {
            memory_manager(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = AMS_SENSOR;
                position_feedback_config.ams_config.factory_settings = 1;
                position_feedback_config.ams_config.polarity = AMS_POLARITY;
                position_feedback_config.ams_config.hysteresis = 1;
                position_feedback_config.ams_config.noise_setting = AMS_NOISE_NORMAL;
                position_feedback_config.ams_config.uvw_abi = 0;
                position_feedback_config.ams_config.dyn_angle_comp = 0;
                position_feedback_config.ams_config.data_select = 0;
                position_feedback_config.ams_config.pwm_on = AMS_PWM_OFF;
                position_feedback_config.ams_config.abi_resolution = 0;
                position_feedback_config.ams_config.resolution_bits = AMS_RESOLUTION;
                position_feedback_config.ams_config.offset = AMS_OFFSET;
                position_feedback_config.ams_config.max_ticks = 0x7fffffff;
                position_feedback_config.ams_config.pole_pairs = 5;
                position_feedback_config.ams_config.cache_time = AMS_CACHE_TIME;
                position_feedback_config.ams_config.velocity_loop = AMS_VELOCITY_LOOP;
                position_feedback_config.ams_config.enable_push_service = PushAll;

                position_feedback_service(null, null, spi_ports,
                                          position_feedback_config, i_shared_memory[0], i_position_feedback,
                                          null, null, null);
            }
        }
    }

    return 0;
}

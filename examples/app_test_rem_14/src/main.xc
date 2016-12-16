/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


/**
 * @file app_test_rem_14.xc
 * @brief Test illustrates usage of the REM_14 rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <position_feedback_service.h>
#include <ctype.h>


SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

/* Test REM_14 Sensor Client */
void rem_14_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;
    timer t;
    unsigned int start_time, end_time, time;


    while(1) {
        /* get position from REM_14 Sensor */
        {count, position} = i_position_feedback.get_position();

        /* get angle from REM_14 Sensor */
        electrical_angle = i_position_feedback.get_angle();

        /* get velocity from REM_14 Sensor */
        velocity = i_position_feedback.get_velocity();

        t :> start_time;
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

    par {
        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: par {
            /* Test REM_14 Encoder Client */
            rem_14_test(i_position_feedback[0], null);

            /* Shared memory Service */
            [[distribute]] memory_manager(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = REM_14_SENSOR;
                position_feedback_config.polarity    = 1;
                position_feedback_config.pole_pairs  = 2;
                position_feedback_config.resolution  = 16384;
                position_feedback_config.offset      = 0;
                position_feedback_config.enable_push_service = PushAll;

                position_feedback_config.rem_14_config.factory_settings = 1;
                position_feedback_config.rem_14_config.hysteresis = 1;
                position_feedback_config.rem_14_config.noise_setting = REM_14_NOISE_NORMAL;
                position_feedback_config.rem_14_config.uvw_abi = 0;
                position_feedback_config.rem_14_config.dyn_angle_comp = 0;
                position_feedback_config.rem_14_config.data_select = 0;
                position_feedback_config.rem_14_config.pwm_on = REM_14_PWM_OFF;
                position_feedback_config.rem_14_config.abi_resolution = 0;
                position_feedback_config.rem_14_config.max_ticks = 0x7fffffff;
                position_feedback_config.rem_14_config.cache_time = REM_14_CACHE_TIME;
                position_feedback_config.rem_14_config.velocity_loop = REM_14_VELOCITY_LOOP;

                position_feedback_service(null, null, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

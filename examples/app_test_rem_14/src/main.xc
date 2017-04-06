/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


/**
 * @file app_test_rem_14.xc
 * @brief Test illustrates usage of the REM_14 rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <position_feedback_service.h>
#include <user_config.h>
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
    int angle = 0;
    timer t;
    unsigned int start_time, end_time, time;


    while(1) {
        /* get position from REM_14 Sensor */
        {count, position, void } = i_position_feedback.get_position();

        /* get angle from REM_14 Sensor */
        angle = i_position_feedback.get_angle();

        /* get velocity from REM_14 Sensor */
        velocity = i_position_feedback.get_velocity();

        t :> start_time;
        if (!isnull(i_shared_memory)) {
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            angle = upstream_control_data.angle;
            count = upstream_control_data.position;
            velocity = upstream_control_data.velocity;
        }
        t :> end_time;

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, angle);
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
            [[distribute]] shared_memory_service(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = REM_14_SENSOR;
                position_feedback_config.resolution  = REM_14_SENSOR_RESOLUTION;
                position_feedback_config.velocity_compute_period = REM_14_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = HOME_OFFSET;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.rem_14_config.hysteresis              = REM_14_SENSOR_HYSTERESIS;
                position_feedback_config.rem_14_config.noise_settings          = REM_14_SENSOR_NOISE_SETTINGS;
                position_feedback_config.rem_14_config.dyn_angle_error_comp    = REM_14_DYN_ANGLE_ERROR_COMPENSATION;
                position_feedback_config.rem_14_config.abi_resolution_settings = REM_14_ABI_RESOLUTION_SETTINGS;

                position_feedback_service(null, null, null, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

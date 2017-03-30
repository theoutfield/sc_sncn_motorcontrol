/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//Hall libs
#include <position_feedback_service.h>
#include <user_config.h>

/* Test Hall Sensor Client */
void hall_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int angle = 0;
    int velocity = 0;
    int count = 0;

    while(1)
    {
        /* get position from Hall Sensor */
        { count, void, void } = i_position_feedback.get_position();
        angle = i_position_feedback.get_angle();

        /* get velocity from Hall Sensor */
        velocity = i_position_feedback.get_velocity();

        if (!isnull(i_shared_memory)) {
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            angle = upstream_control_data.angle;
            count = upstream_control_data.position;
            velocity = upstream_control_data.velocity;
        }

        xscope_int(COUNT, count);
        xscope_int(VELOCITY, velocity);
        xscope_int(ANGLE, angle);
    }
}

QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;

int main(void)
{
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]: par {
            /* Test Hall sensor Client */
            hall_test(i_position_feedback[0], i_shared_memory[1]);

            /* Shared memory Service */
            [[distribute]] shared_memory_service(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = HALL_SENSOR;
                position_feedback_config.resolution  = HALL_SENSOR_RESOLUTION;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.velocity_compute_period = HALL_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.offset      = 0;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, null, null, null, null, null, null,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

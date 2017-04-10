/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

/**
 * @file test_qei.xc
 * @brief Test illustrates usage of qei sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <position_feedback_service.h>
#include <user_config.h>

/* Test QEI Sensor Client */
void qei_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int position, velocity, count;

    while(1)
    {
        /* get position and velocity from QEI Sensor */
        { count, position, void } = i_position_feedback.get_position();

        velocity = i_position_feedback.get_velocity();

        if (!isnull(i_shared_memory)) {
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            count = upstream_control_data.position;
            velocity = upstream_control_data.velocity;
        }

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}

QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;

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
            /* Test QEI Sensor Client */
            qei_test(i_position_feedback[0], null);

            /* Shared memory Service */
            [[distribute]] shared_memory_service(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = QEI_SENSOR;
                position_feedback_config.polarity    = NORMAL_POLARITY;
                position_feedback_config.resolution  = QEI_SENSOR_RESOLUTION;
                position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config.velocity_compute_period = QEI_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                position_feedback_config.qei_config.number_of_channels = QEI_SENSOR_NUMBER_OF_CHANNELS;
                position_feedback_config.qei_config.signal_type        = QEI_SENSOR_SIGNAL_TYPE;
                position_feedback_config.qei_config.port_number        = QEI_SENSOR_PORT_NUMBER;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, null, null, null, null, null,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

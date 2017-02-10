/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

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
            { void, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}

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
            [[distribute]] memory_manager(i_shared_memory, 2);

            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = QEI_SENSOR;
                position_feedback_config.polarity    = FEEDBACK_SENSOR_POLARITY;
                position_feedback_config.resolution  = FEEDBACK_SENSOR_RESOLUTION;
                position_feedback_config.enable_push_service = PushAll;

                position_feedback_config.qei_config.index_type = QEI_SENSOR_INDEX_TYPE;
                position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;

                position_feedback_service(null, qei_hall_port_2, hall_enc_select_port, null, null, null, null, null,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        null, null, null);
            }
        }
    }

    return 0;
}

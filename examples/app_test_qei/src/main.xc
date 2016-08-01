/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_qei.xc
 * @brief Test illustrates usage of qei sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <position_feedback_service.h>

/* Test QEI Sensor Client */
void qei_test(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
    int position, velocity, count, valid;

    while(1)
    {
        /* get position and velocity from QEI Sensor */
        { count, position } = i_position_feedback.get_position();

        velocity = i_position_feedback.get_velocity();

        if (!isnull(i_shared_memory)) {
            { void, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);

        //printf("Position: %d Velocity: %d\n", position, velocity);

        delay_milliseconds(1);
    }
}

QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

int main(void)
{
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        on tile[APP_TILE]:
        {
            /* Test QEI Sensor Client */
            qei_test(i_position_feedback[0], null);
        }

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]: par {
            memory_manager(i_shared_memory, 2);


            /* Position feedback service */
            {
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.sensor_type = QEI_SENSOR;
                position_feedback_config.qei_config.ticks_resolution = 1000;
                position_feedback_config.qei_config.index_type = QEI_WITH_INDEX;
                position_feedback_config.qei_config.sensor_polarity = 1;
                position_feedback_config.qei_config.signal_type = QEI_RS422_SIGNAL;
                position_feedback_config.qei_config.enable_push_service = PushPosition;

                position_feedback_service(null, qei_ports, null,
                                         position_feedback_config, i_shared_memory[0], i_position_feedback,
                                         null, null, null);
            }
        }
    }

    return 0;
}

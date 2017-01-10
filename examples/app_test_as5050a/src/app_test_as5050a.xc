#include <CORE_C22-rev-a.bsp>
//#include <IFM_DC100-rev-b.bsp>
#include <position_feedback_service.h>

SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;

void as5050a_runtest(client interface PositionFeedbackInterface i_position_feedback, client interface shared_memory_interface ?i_shared_memory)
{
            int count = 0;
            int velocity = 0;
            int position = 0;
            int electrical_angle = 0;
            timer t;
            unsigned int start_time, end_time, time;


            while(1) {
                /* get position from REM_10 Sensor */
                {count, position} = i_position_feedback.get_position();

                /* get angle from REM_10 Sensor */
                electrical_angle = i_position_feedback.get_angle();

                /* get velocity from REM_10 Sensor */
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
//                xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds
//                xscope_int(TIME_INTERNAL, time);   //time to get the data in microseconds

                delay_milliseconds(1);
            }
}

int main(void)
{
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];


    par {
            on tile[IFM_TILE]: par {
                /* Test REM_10 Encoder Client */
                as5050a_runtest(i_position_feedback[0], null);

                /* Shared memory Service */
                [[distribute]] memory_manager(i_shared_memory, 2);

                /* Position feedback service */
                {
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = REM_10_SENSOR;
                    position_feedback_config.polarity    = 1;
                    position_feedback_config.pole_pairs  = 2;
                    position_feedback_config.resolution  = 1024;
                    position_feedback_config.offset      = 0;
                    position_feedback_config.enable_push_service = PushAll;

                    position_feedback_config.as5050a_config.max_ticks = 0x7fffffff;
                    position_feedback_config.as5050a_config.velocity_loop = 1000;

                    position_feedback_service(null, null, spi_ports, position_feedback_config, i_shared_memory[0], i_position_feedback, null, null, null);
                }
            }
        }
    return 0;
}

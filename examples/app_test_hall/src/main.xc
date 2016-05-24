/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//Hall libs
#include <hall_service.h>

/* Test Hall Sensor Client */
void hall_test(interface HallInterface client i_hall, client interface shared_memory_interface ?i_shared_memory)
{
    int position = 0;
    int velocity = 0;
    int count = 0;

    xscope_int(COUNT, count);
    xscope_int(VELOCITY, velocity);

    while(1)
    {
        /* get position from Hall Sensor */
        count = i_hall.get_hall_position_absolute();
        position = i_hall.get_hall_position();

        /* get velocity from Hall Sensor */
        velocity = i_hall.get_hall_velocity();

        if (!isnull(i_shared_memory)) {
            { void, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }

//        printintln(position);

        xscope_int(COUNT, count);
        xscope_int(VELOCITY, velocity);
    }
}

HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;

int main(void)
{
    interface HallInterface i_hall[5];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        /* Client side */
        on tile[APP_TILE]: hall_test(i_hall[0], null);

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]: par {
        memory_manager(i_shared_memory, 2);

        /* Hall Service */
        {
            HallConfig hall_config;
            hall_config.pole_pairs = 1;
            hall_config.enable_push_service = PushAll;

            hall_service(hall_ports, hall_config, i_shared_memory[0], i_hall);
        }
        }
    }

    return 0;
}

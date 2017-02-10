/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


/**
 * @file main.xc
 * @brief Test illustrates usage of position feedback service to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//libs
#include <position_feedback_service.h>
#include <ctype.h>
#include <stdio.h>

/* Test Sensor Client */
void position_feedback_test(client interface PositionFeedbackInterface i_position_feedback_1,
                            client interface PositionFeedbackInterface ?i_position_feedback_2,
                            client interface shared_memory_interface ?i_shared_memory)
{
    int angle = 0;
    int velocity = 0;
    int count = 0;
    int angle_2 = 0;
    int velocity_2 = 0;
    int count_2 = 0;

    while(1)
    {
        /* get position from Hall Sensor */
        { count, void, void } = i_position_feedback_1.get_position();
        angle = i_position_feedback_1.get_angle();

        /* get velocity from Hall Sensor */
        velocity = i_position_feedback_1.get_velocity();

        if (!isnull(i_position_feedback_2)) {
            { count_2, void, void } = i_position_feedback_2.get_position();
            angle_2 = i_position_feedback_2.get_angle();
            velocity_2 = i_position_feedback_2.get_velocity();
        }

        if (!isnull(i_shared_memory)) {
            { angle, velocity, count } = i_shared_memory.get_angle_velocity_position();
            SharedMemoryData data = i_shared_memory.read();
            unsigned int gpio_out = 0b1010;
            i_shared_memory.write_gpio_output(gpio_out);
            xscope_int(GPIO_0, 1000 * data.gpio[0]);
        }

        xscope_int(COUNT_1, count);
        xscope_int(VELOCITY_1, velocity);
        xscope_int(ANGLE_1, angle);
        xscope_int(COUNT_2, count_2);
        xscope_int(VELOCITY_2, velocity_2);
        xscope_int(ANGLE_2, angle_2);

        delay_milliseconds(1);
    }
}

void commands_test(client interface PositionFeedbackInterface i_position_feedback_1, client interface PositionFeedbackInterface ?i_position_feedback_2) {

    delay_milliseconds(500);
    PositionFeedbackConfig position_feedback_config_1 = i_position_feedback_1.get_config();
    PositionFeedbackConfig position_feedback_config_2;
    if (!isnull(i_position_feedback_2))
        position_feedback_config_2 = i_position_feedback_2.get_config();

    printstr(">>   SOMANET POSITON FEEDBACK COMMANDS SERVICE STARTING...\n");

    while(1) {
        char mode = 0;
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            } else if (c == '-') {
                sign = -1;
            } else if (c != ' ')
                mode = c;
        }

        switch(mode) {
        //exit
        case 'e':
            i_position_feedback_1.exit();
            if (!isnull(i_position_feedback_2))
                i_position_feedback_2.exit();
            printf("exit\n");
            break;
        //set sensor 1
        case 'a':
            position_feedback_config_1.sensor_type = value;
            i_position_feedback_1.set_config(position_feedback_config_1);
            printf("sensor 1 set to %d\n", position_feedback_config_1.sensor_type);
            break;
        //set sensor 2
        case 'b':
            position_feedback_config_2.sensor_type = value;
            i_position_feedback_2.set_config(position_feedback_config_2);
            printf("sensor 2 set to %d\n", position_feedback_config_2.sensor_type);
            break;
        }
        delay_milliseconds(10);
    }
}

QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

int main(void)
{
    interface PositionFeedbackInterface i_position_feedback[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];
    interface shared_memory_interface i_shared_memory[3];

    par
    {
        /* Client side */
        on tile[APP_TILE]: commands_test(i_position_feedback[1], i_position_feedback_2[1]);

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]: par {
            position_feedback_test(i_position_feedback[0], i_position_feedback_2[0], i_shared_memory[1]);

            /* Shared memory Service */
            [[distribute]] memory_manager(i_shared_memory, 3);

            /* Position feedback service */
            {
                //set default parameters
                PositionFeedbackConfig position_feedback_config;
                position_feedback_config.polarity    = 1;
                position_feedback_config.pole_pairs  = 2;
                position_feedback_config.offset      = 0;
                position_feedback_config.enable_push_service = PushAll;

                position_feedback_config.biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config.biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                position_feedback_config.biss_config.status_length = BISS_STATUS_LENGTH;
                position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config.biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                position_feedback_config.biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config.biss_config.max_ticks = BISS_MAX_TICKS;
                position_feedback_config.biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT_EXT_D5;
                position_feedback_config.biss_config.data_port_config = BISS_DATA_PORT_2;

                position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;
                position_feedback_config.rem_16mt_config.timeout = REM_16MT_TIMEOUT;
                position_feedback_config.rem_16mt_config.velocity_loop = REM_16MT_VELOCITY_LOOP;

                position_feedback_config.qei_config.index_type = QEI_SENSOR_INDEX_TYPE;
                position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;

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

                position_feedback_config.gpio_config[0] = GPIO_INPUT_PULLDOWN;
                position_feedback_config.gpio_config[1] = GPIO_OUTPUT;
                position_feedback_config.gpio_config[2] = GPIO_OUTPUT;
                position_feedback_config.gpio_config[3] = GPIO_OUTPUT;

                PositionFeedbackConfig position_feedback_config_2;
                position_feedback_config_2 = position_feedback_config;

                //set sensor 1 parameters
                position_feedback_config.sensor_type = HALL_SENSOR;
                position_feedback_config.resolution  = 8192;

                //set sensor 1 parameters
                position_feedback_config_2.sensor_type = BISS_SENSOR;
                position_feedback_config_2.resolution  = 8192;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config, i_shared_memory[0], i_position_feedback,
                        position_feedback_config_2, null, i_position_feedback_2);
            }
        }
    }

    return 0;
}

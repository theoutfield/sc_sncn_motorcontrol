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
#include <user_config.h>
#include <ctype.h>
#include <stdio.h>

/* Test Sensor Client */
void position_feedback_display(client interface PositionFeedbackInterface i_position_feedback_1,
                            client interface PositionFeedbackInterface ?i_position_feedback_2,
                            client interface shared_memory_interface ?i_shared_memory)
{
    int angle_1, velocity_1, position_1;
    int angle_2, velocity_2, position_2;

    while(1)
    {
        /* get position and velocity from sensor 1 */
        { position_1, void, void } = i_position_feedback_1.get_position();
        angle_1 = i_position_feedback_1.get_angle();
        velocity_1 = i_position_feedback_1.get_velocity();

        /* get position and velocity from sensor 2 */
        if (!isnull(i_position_feedback_2)) {
            { position_2, void, void } = i_position_feedback_2.get_position();
            angle_2 = i_position_feedback_2.get_angle();
            velocity_2 = i_position_feedback_2.get_velocity();
        }

        // get position,velocity and gpio from shared memory
        if (!isnull(i_shared_memory)) {
            //get position data
            UpstreamControlData upstream_control_data = i_shared_memory.read();
            xscope_int(ANGLE_SHARED_MEMORY, upstream_control_data.angle);
            xscope_int(POSITION_SHARED_MEMORY, upstream_control_data.position);
            xscope_int(POSITION_SECONDARY_SHARED_MEMORY, upstream_control_data.secondary_position);
            xscope_int(VELOCITY_SHARED_MEMORY, upstream_control_data.velocity);

            //write gpio
            unsigned int gpio_out = 0b0010; // we write 1 to the port 1 and 0 to the port 3, port 0 and 2 are in read mode
            i_shared_memory.write_gpio_output(gpio_out);

            //read gpio
            xscope_int(GPIO_0, 1000 * upstream_control_data.gpio[0]);
            xscope_int(GPIO_2, 1000 * upstream_control_data.gpio[2]);
        }

        xscope_int(POSITION_1, position_1);
        xscope_int(VELOCITY_1, velocity_1);
        xscope_int(ANGLE_1, angle_1);
        xscope_int(POSITION_2, position_2);
        xscope_int(VELOCITY_2, velocity_2);
        xscope_int(ANGLE_2, angle_2);

        delay_milliseconds(1);
    }
}

void position_feedback_settings(PositionFeedbackConfig &position_feedback_config, int sensor, char mode, int value)
{
    switch(mode)
    {
    //resolution
    case 'r':
        position_feedback_config.resolution = value;
        printf("set sensor %d resolution to %d\n", sensor, position_feedback_config.resolution);
        break;
    //velocity compute period
    case 'v':
        position_feedback_config.velocity_compute_period = value;
        printf("set sensor %d velocity compute period to %d microseconds\n", sensor, position_feedback_config.velocity_compute_period);
        break;
    case 'p':
        EncoderPortNumber port_number;
        if (value == 2) {
            port_number = ENCODER_PORT_2;
        } else {
            port_number = ENCODER_PORT_1;
            value = 1;
        }
        position_feedback_config.biss_config.data_port_number = port_number;
        position_feedback_config.hall_config.port_number = port_number;
        position_feedback_config.qei_config.port_number = port_number;
        printf("set sensor %d on port %d\n", sensor, value);
        break;
    //set sensor type
    default:
        position_feedback_config.sensor_type = value;
        printf("set sensor %d type to %d\n", sensor, position_feedback_config.sensor_type);
        // auto set resolution and velocity_compute_period
        int old_resolution = position_feedback_config.resolution;
        int old_velocity_compute_period = position_feedback_config.velocity_compute_period;
        switch(position_feedback_config.sensor_type)
        {
        case HALL_SENSOR:
            position_feedback_config.resolution = HALL_SENSOR_RESOLUTION;
            position_feedback_config.velocity_compute_period = HALL_SENSOR_VELOCITY_COMPUTE_PERIOD;
            break;
        case QEI_SENSOR:
            position_feedback_config.resolution = QEI_SENSOR_RESOLUTION;
            position_feedback_config.velocity_compute_period = QEI_SENSOR_VELOCITY_COMPUTE_PERIOD;
            break;
        case BISS_SENSOR:
            position_feedback_config.resolution = BISS_SENSOR_RESOLUTION;
            position_feedback_config.velocity_compute_period = BISS_SENSOR_VELOCITY_COMPUTE_PERIOD;
            break;
        case REM_14_SENSOR:
            position_feedback_config.resolution = REM_14_SENSOR_RESOLUTION;
            position_feedback_config.velocity_compute_period = REM_14_SENSOR_VELOCITY_COMPUTE_PERIOD;
            break;
        case REM_16MT_SENSOR:
            position_feedback_config.resolution = REM_16MT_SENSOR_RESOLUTION;
            position_feedback_config.velocity_compute_period = REM_16MT_SENSOR_VELOCITY_COMPUTE_PERIOD;
            break;
        }
        if (old_resolution != position_feedback_config.resolution) {
            printf("auto set sensor %d resolution to %d\n", sensor, position_feedback_config.resolution);
        }
        if (old_velocity_compute_period != position_feedback_config.velocity_compute_period) {
            printf("auto set sensor %d velocity compute period to %d microseconds\n", sensor, position_feedback_config.velocity_compute_period);
        }
        break;
    }
}

void position_feedback_commands(client interface PositionFeedbackInterface i_position_feedback_1, client interface PositionFeedbackInterface ?i_position_feedback_2) {

    delay_milliseconds(500);
    PositionFeedbackConfig position_feedback_config_1 = i_position_feedback_1.get_config();
    PositionFeedbackConfig position_feedback_config_2;
    if (!isnull(i_position_feedback_2))
        position_feedback_config_2 = i_position_feedback_2.get_config();

    printstrln(">>   SOMANET POSITON FEEDBACK COMMANDS SERVICE STARTING...");
    printstrln("Commands:\n"
               "a [number]: set first sensor type\n"
               "b [number]: set second sensor type\n"
               "ar [number]: set first sensor resolution (ticks per turn)\n"
               "av [number]: set first sensor velocity compute period in microseconds\n"
               "ap [number]: set first sensor port number (1 or 2)\n"
               "(same commands with 'b' for sensor 2)\n"
               "e: exit and restart the position feedback service\n"
               "The new sensor type is taken into account when the service is restarted");

    while(1) {
        char mode_1 = 0, mode_2=0;
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
            } else if (c != ' ') {
                if (mode_1 == 0) {
                    mode_1 = c;
                } else {
                    mode_2 = c;
                }
            }
        }

        switch(mode_1) {
        //exit
        case 'e':
            i_position_feedback_1.exit();
            if (!isnull(i_position_feedback_2))
                i_position_feedback_2.exit();
            printf("exit\n");
            break;
        //set sensor 1
        case 'a':
            position_feedback_settings(position_feedback_config_1, 1, mode_2, value);
            i_position_feedback_1.set_config(position_feedback_config_1);
            break;
        //set sensor 2
        case 'b':
            position_feedback_settings(position_feedback_config_2, 2, mode_2, value);
            i_position_feedback_2.set_config(position_feedback_config_2);
            break;
        }
        delay_milliseconds(10);
    }
}

SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_ENCODER_PORTS_INPUT_MODE_SELECTION;
port ? qei_hall_port_1 = SOMANET_IFM_ENCODER_1_PORT;
port ? qei_hall_port_2 = SOMANET_IFM_ENCODER_2_PORT;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

int main(void)
{
    interface PositionFeedbackInterface i_position_feedback_1[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];
    interface shared_memory_interface i_shared_memory[3];

    par
    {
        /* Client side */
        on tile[APP_TILE]: position_feedback_commands(i_position_feedback_1[1], i_position_feedback_2[1]);

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IF2_TILE]: par {
            position_feedback_display(i_position_feedback_1[0], i_position_feedback_2[0], i_shared_memory[2]);

            /* Shared memory Service */
            [[distribute]] shared_memory_service(i_shared_memory, 3);

            /* Position feedback service */
            {
                //set default parameters
                PositionFeedbackConfig position_feedback_config_1;
                position_feedback_config_1.polarity    = SENSOR_POLARITY_NORMAL;
                position_feedback_config_1.pole_pairs  = MOTOR_POLE_PAIRS;
                position_feedback_config_1.ifm_usec    = IF2_TILE_USEC;
                position_feedback_config_1.max_ticks   = SENSOR_MAX_TICKS;
                position_feedback_config_1.offset      = HOME_OFFSET;

                position_feedback_config_1.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                position_feedback_config_1.biss_config.filling_bits = BISS_FILLING_BITS;
                position_feedback_config_1.biss_config.crc_poly = BISS_CRC_POLY;
                position_feedback_config_1.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                position_feedback_config_1.biss_config.timeout = BISS_TIMEOUT;
                position_feedback_config_1.biss_config.busy = BISS_BUSY;
                position_feedback_config_1.biss_config.clock_port_config = BISS_CLOCK_PORT;
                position_feedback_config_1.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;
                position_feedback_config_1.biss_config.data_port_signal_type = BISS_DATA_PORT_SIGNAL_TYPE;

                position_feedback_config_1.rem_16mt_config.filter = REM_16MT_FILTER;

                position_feedback_config_1.rem_14_config.hysteresis              = REM_14_SENSOR_HYSTERESIS;
                position_feedback_config_1.rem_14_config.noise_settings          = REM_14_SENSOR_NOISE_SETTINGS;
                position_feedback_config_1.rem_14_config.dyn_angle_error_comp    = REM_14_DYN_ANGLE_ERROR_COMPENSATION;
                position_feedback_config_1.rem_14_config.abi_resolution_settings = REM_14_ABI_RESOLUTION_SETTINGS;

                position_feedback_config_1.qei_config.number_of_channels = QEI_SENSOR_NUMBER_OF_CHANNELS;
                position_feedback_config_1.qei_config.signal_type        = QEI_SENSOR_SIGNAL_TYPE;
                position_feedback_config_1.qei_config.port_number        = QEI_SENSOR_PORT_NUMBER;
                position_feedback_config_1.qei_config.ticks_lost_threshold = QEI_SENSOR_TICKS_LOST;

                position_feedback_config_1.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;
                position_feedback_config_1.hall_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                position_feedback_config_1.hall_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                position_feedback_config_1.hall_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                position_feedback_config_1.hall_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                position_feedback_config_1.hall_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                position_feedback_config_1.hall_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;

                position_feedback_config_1.gpio_config[0] = GPIO_INPUT;
                position_feedback_config_1.gpio_config[1] = GPIO_OUTPUT;
                position_feedback_config_1.gpio_config[2] = GPIO_INPUT_PULLDOWN;
                position_feedback_config_1.gpio_config[3] = GPIO_OUTPUT;

                PositionFeedbackConfig position_feedback_config_2;
                position_feedback_config_2 = position_feedback_config_1;

                //set sensor 1 parameters
                position_feedback_config_1.sensor_type = HALL_SENSOR;
                position_feedback_config_1.resolution  = HALL_SENSOR_RESOLUTION;
                position_feedback_config_1.velocity_compute_period = HALL_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config_1.sensor_function = SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL;

                //set sensor 1 parameters
                position_feedback_config_2.sensor_type = BISS_SENSOR;
                position_feedback_config_2.resolution  = BISS_SENSOR_RESOLUTION;
                position_feedback_config_2.velocity_compute_period = BISS_SENSOR_VELOCITY_COMPUTE_PERIOD;
                position_feedback_config_2.sensor_function = SENSOR_FUNCTION_FEEDBACK_DISPLAY_ONLY;

                position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                        position_feedback_config_1, i_shared_memory[0], i_position_feedback_1,
                        position_feedback_config_2, i_shared_memory[1], i_position_feedback_2);
            }
        }
    }

    return 0;
}

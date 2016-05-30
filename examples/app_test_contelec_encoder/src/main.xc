/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file app_test_contelec_rotary_sensor.xc
 * @brief Test illustrates usage of the CONTELEC rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <contelec_service.h>
#include <position_service.h>
#include <ctype.h>
#include <stdio.h>


//SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
PositionPorts position_ports = { {null, null, null}, SOMANET_IFM_AMS_PORTS};


/* Test CONTELEC Sensor Client */
void contelec_encoder_test(client interface PositionInterface i_position, client interface shared_memory_interface ?i_shared_memory)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;
    int status = 0;
    timer t;
    unsigned start_time, end_time;

    while(1) {
        /* get position from CONTELEC Sensor */
        t :> start_time;
        {void, void, status} = i_position.get_real_position();
        t :> end_time;
        {count, position} = i_position.get_position();

        /* get angle and velocity from CONTELEC Sensor */
        velocity = i_position.get_velocity();

        if (!isnull(i_shared_memory)) {
            { void, velocity, count } = i_shared_memory.get_angle_velocity_position();
        }


        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
//        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);
//        xscope_int(STATUS, status*1000);
        xscope_int(TIME, status);

        delay_microseconds(10);
    }
}

void contelec_encoder_commands_test(client interface PositionInterface i_position) {
    char status;
    int multiturn;
//    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned start_time, end_time;
    timer t;

    delay_milliseconds(500);
    PositionConfig position_config = i_position.get_config();
    printstr(">>   SOMANET CONTELEC SENSOR COMMANDS SERVICE STARTING...\n");

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
        //set angle
        case 'a':
            printf("Set angle to %d\nnew offset %d\n", value, i_position.set_angle(value));
            break;
        //set calibration point
        case 'c':
            i_position.send_command(0x3E, value, 16);
            printf("set calibration point %d\n", value);
            break;
        //change direction
        case 'd':
            i_position.send_command(0x55, value, 8);
            printf("direction %d\n", value);
            break;
        //filter
        case 'f':
            i_position.send_command(0x5B, value, 8);
            printf("filter %d\n", value);
            break;
        //set offset
        case 'o':
            position_config.contelec_config.offset = value * sign;
            i_position.set_config(position_config);
            printf("offset %d\n", value * sign);
            break;
        //set position
        case 'p':
            i_position.set_position(value*sign);
            break;
        //set multiturn
        case 'm':
            i_position.send_command(0x59, value*sign, 16);
            printf("multiturn\n");
            break;
        //reset sensor
        case 'r':
            i_position.send_command(0x00, 0, 0);
            printf("reset\n");
            break;
        //set singleturn
        case 's':
            i_position.send_command(0x50, value, 16);
            printf("singleturn\n");
            break;
        //calibration table size
        case 't':
            i_position.send_command(0x3D, value, 16);
            printf("calibration table size %d\n", value);
            break;
        //save
        case 'v':
            i_position.send_command(0x1C, 0, 0);
            i_position.send_command(0x00, 0, 0);
            printf("save\n");
            break;
        //set zero position
        case 'z':
            i_position.send_command(0x56, 0, 0);
            printf("zero\n");
            break;
        //print the count and the time to get it
        default:
            t :> start_time;
            {multiturn, singleturn_raw, status} = i_position.get_real_position();
            t :> end_time;
            printf("time %d, count %d\n", (end_time-start_time)/USEC_STD, multiturn);
            break;
        }
        delay_milliseconds(10);
    }
}


int main(void)
{
    interface PositionInterface i_position[3];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        on tile[APP_TILE]: contelec_encoder_commands_test(i_position[1]);

        on tile[IFM_TILE]: par {
            contelec_encoder_test(i_position[0], i_shared_memory[1]);

            /* Shared memory Service */
            memory_manager(i_shared_memory, 2);


            /* CONTELEC Sensor Service */
//            {
//                CONTELECConfig contelec_config;
//                contelec_config.filter = CONTELEC_FILTER;
//                contelec_config.polarity = CONTELEC_POLARITY;
//                contelec_config.resolution_bits = CONTELEC_RESOLUTION;
//                contelec_config.offset = CONTELEC_OFFSET;
//                contelec_config.pole_pairs = 2;
//                contelec_config.timeout = CONTELEC_TIMEOUT;
//                contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
//                contelec_config.enable_push_service = PushAll;
//
//                contelec_service(spi_ports, contelec_config, i_shared_memory[0], i_position);
//            }


            /* Position service */
            {
                PositionConfig position_config;
                position_config.sensor_type[0] = CONTELEC_SENSOR;
                position_config.contelec_config.filter = CONTELEC_FILTER;
                position_config.contelec_config.polarity = CONTELEC_POLARITY;
                position_config.contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                position_config.contelec_config.offset = CONTELEC_OFFSET;
                position_config.contelec_config.pole_pairs = 2;
                position_config.contelec_config.timeout = CONTELEC_TIMEOUT;
                position_config.contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                position_config.contelec_config.enable_push_service = PushAll;

                position_service(position_ports, position_config, i_shared_memory[0], i_position);
            }
        }
    }

    return 0;
}

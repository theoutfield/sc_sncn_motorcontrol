/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
//#include <CORE_BOARD_REQUIRED>
//#include <IFM_BOARD_REQUIRED>
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

/**
 * @file app_test_contelec_rotary_sensor.xc
 * @brief Test illustrates usage of the CONTELEC rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <contelec_service.h>
#include <ctype.h>
#include <stdio.h>


SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;

/* Test CONTELEC Sensor Client */
void contelec_encoder_test(client interface CONTELECInterface i_contelec)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;

    while(1) {
        /* get position from CONTELEC Sensor */
        {count, position} = i_contelec.get_contelec_position();

        /* get angle and velocity from CONTELEC Sensor */
        { electrical_angle, velocity } = i_contelec.get_contelec_angle_velocity();

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}

void contelec_encoder_commands_test(client interface CONTELECInterface i_contelec) {
    char status;
    int multiturn;
//    unsigned int singleturn_filtered;
//    unsigned int singleturn_raw;
    unsigned start_time, end_time;
    timer t;

    delay_milliseconds(500);
    CONTELECConfig contelec_config = i_contelec.get_contelec_config();
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
            printf("Set angle to %d\nnew offset %d\n", value, i_contelec.reset_contelec_angle(value));
            break;
        //change direction
        case 'd':
            i_contelec.command_contelec(0x55, value, 8);
            printf("direction\n");
            break;
        //set offset
        case 'o':
            contelec_config.offset = value * sign;
            i_contelec.set_contelec_config(contelec_config);
            printf("offset %d\n", value * sign);
            break;
        //set position
        case 'p':
            i_contelec.reset_contelec_position(value*sign);
            break;
        //set multiturn
        case 'm':
            i_contelec.command_contelec(0x59, value*sign, 16);
            printf("multiturn\n");
            break;
        //reset sensor
        case 'r':
            i_contelec.command_contelec(0x00, 0, 0);
            printf("reset\n");
            break;
        //set singleturn
        case 's':
            i_contelec.command_contelec(0x57, value, 16);
            printf("singleturn\n");
            break;
        //set zero position
        case 'z':
            i_contelec.command_contelec(0x56, 0, 0);
            printf("zero\n");
            break;
        //print the count and the time to get it
        default:
            t :> start_time;
            {multiturn, status} = i_contelec.get_contelec_real_position();
            t :> end_time;
            printf("time %d, count %d\n", (end_time-start_time)/USEC_STD, multiturn);
            break;
        }
        delay_milliseconds(10);
    }
}


int main(void)
{
    interface CONTELECInterface i_contelec[5];

    par
    {
        on tile[APP_TILE]: contelec_encoder_commands_test(i_contelec[1]);

        on tile[IFM_TILE]: par {
            contelec_encoder_test(i_contelec[0]);


            /* CONTELEC Sensor Service */
            {
                CONTELECConfig contelec_config;
                contelec_config.filter = CONTELEC_FILTER;
                contelec_config.polarity = CONTELEC_POLARITY;
                contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                contelec_config.offset = CONTELEC_OFFSET;
                contelec_config.pole_pairs = 2;
                contelec_config.timeout = CONTELEC_TIMEOUT;
                contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;

                contelec_service(spi_ports, contelec_config, i_contelec);
            }
        }
    }

    return 0;
}

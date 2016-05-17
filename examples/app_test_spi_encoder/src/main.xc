/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <ams_service.h>
#include <ctype.h>
#include <stdio.h>


AMSPorts ams_ports = SOMANET_IFM_AMS_PORTS;

/* Test AMS Sensor Client */
void ams_rotary_sensor_test(client interface AMSInterface i_ams)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;

    while(1) {
        /* get position from AMS Sensor */
        {count, position} = i_ams.get_ams_position();

        /* get angle and velocity from AMS Sensor */
        { electrical_angle, velocity } = i_ams.get_ams_angle_velocity();

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}

void spi_encoder_test(client interface AMSInterface i_ams) {
    char status;
    int multiturn;
    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;

    delay_milliseconds(500);
    AMSConfig ams_config = i_ams.get_ams_config();
    printf("Start!\n");

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
        case 'a':
            printf("Set angle to %d\nnew offset %d\n", value, i_ams.reset_ams_angle(value));
            break;
        case 'd':
            i_ams.command_ams(0x55, value, 8);
            printf("direction\n");
            break;
        case 'o':
            ams_config.offset = value * sign;
            i_ams.set_ams_config(ams_config);
            printf("offset %d\n", value * sign);
            break;
        case 'p':
            i_ams.reset_ams_position(value*sign);
            break;
        case 'm':
            i_ams.command_ams(0x59, value*sign, 16);
            printf("multiturn\n");
            break;
        case 'r':
            i_ams.command_ams(0x00, 0, 0);
            printf("reset\n");
            break;
        case 's':
            i_ams.command_ams(0x57, value, 16);
            printf("singleturn\n");
            break;
        case 'z':
            i_ams.command_ams(0x56, 0, 0);
            printf("zero\n");
            break;
//        default:
////            { status, multiturn, singleturn_filtered, singleturn_raw } = spi_encoder_read(ams_ports);
//            printf("status 0x%X, multiturn %d\nsingleturn_f %d, singleturn_r %d\n", status, multiturn, singleturn_filtered, singleturn_raw);
//        break;
        }

//        ams_ports.slave_select <: 0; // Ensure slave select is in correct start state
//        delay_ticks(10*100);
//        ams_ports.slave_select <: 1;

        delay_milliseconds(10);
    }
}


int main(void)
{
    interface AMSInterface i_ams[5];

    par
    {
//        on tile[APP_TILE]: spi_encoder_test();

        on tile[APP_TILE]: spi_encoder_test(i_ams[1]);

        on tile[IFM_TILE]: par {
            ams_rotary_sensor_test(i_ams[0]);


            /* AMS Rotary Sensor Service */
            {
                AMSConfig ams_config;
                ams_config.sensor_type = CONTELEC_SENSOR;
                ams_config.filter = 0x02;
                ams_config.factory_settings = 1;
                ams_config.polarity = AMS_POLARITY;
                ams_config.hysteresis = 1;
                ams_config.noise_setting = AMS_NOISE_NORMAL;
                ams_config.uvw_abi = 0;
                ams_config.dyn_angle_comp = 0;
                ams_config.data_select = 0;
                ams_config.pwm_on = AMS_PWM_OFF;
                ams_config.abi_resolution = 0;
                ams_config.resolution_bits = AMS_RESOLUTION;
                ams_config.offset = AMS_OFFSET;
                ams_config.pole_pairs = 2;
                ams_config.max_ticks = 0x7fffffff;
                ams_config.cache_time = AMS_CACHE_TIME;
                ams_config.velocity_loop = AMS_VELOCITY_LOOP;

                ams_service(ams_ports, ams_config, i_ams);
            }
        }
    }

    return 0;
}

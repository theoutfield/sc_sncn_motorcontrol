/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC300-rev-a.bsp>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <ams_service.h>
#include <user_config.h>
#include <ctype.h>

#define NUM_OF_AMS_INTERFACES 2

//DC1K
on tile[IFM_TILE]: sensor_spi_interface pRotarySensor =
{
        {
            XS1_CLKBLK_2,
            XS1_CLKBLK_4,
            SOMANET_IFM_GPIO_D3, //D3,    //mosi
            SOMANET_IFM_GPIO_D1, //D1,    //sclk
            SOMANET_IFM_GPIO_D2  //D2     //miso
        },

        SOMANET_IFM_GPIO_D0 //D0         //slave select
};





/* Test Hall Sensor Client */
void ams_rotary_sensor_test(client interface AMSInterface i_ams)
{

    int position = 0;
//    int velocity = 0;
    int direction = 0;
    int electrical_angle = 0;

    while(1)
    {

        electrical_angle = i_ams.get_ams_angle_electrical();

        /* get position from Hall Sensor */
        {position, direction} = i_ams.get_ams_position();
//        position = i_ams.get_ams_real_position();

        /* get velocity from Hall Sensor */
   //     velocity = i_ams.get_ams_velocity();

        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);

        delay_milliseconds(1);
   //     printf("%i\n", position);

    }
}

void ams_rotary_sensor_direct_method(sensor_spi_interface &sensor_if, unsigned short settings1, unsigned short settings2, unsigned short offset){
    int result = initRotarySensor(sensor_if,  settings1,  settings2, offset);
    int abs_position = 0;

    printf("result init: %d\n",result);
    printf("pole pairs init: %d\n", readNumberPolePairs(sensor_if));

    while(1){
        abs_position = readRotarySensorAngleWithoutCompensation(sensor_if);
        xscope_int(POSITION, abs_position);
        printf("%i\n", abs_position);
    }


}

void test(client interface AMSInterface i_ams) {
    delay_seconds(2);
    printf("test\n");
    fflush(stdout);
    while (1) {
        char c;
        int value = 0;
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            }
        }
        int offset = i_ams.reset_ams_angle_electrical(value);
        printf("offset %d\n", offset);
    }
}



int main(void)
{
    interface AMSInterface i_ams[5];

    par
    {
        on tile[APP_TILE]: test(i_ams[1]);

        on tile[COM_TILE]:
        {
//            i_ams[0].configure(set_configuration());
            ams_rotary_sensor_test(i_ams[0]);

        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            /* AMS Rotary Sensor Server */
            AMSConfig ams_config;

            ams_config.settings1 = AMS_INIT_SETTINGS1;
            ams_config.settings2 = AMS_INIT_SETTINGS2;
            ams_config.resolution_bits = ROTARY_SENSOR_RESOLUTION_BITS;
            ams_config.offset_electrical = SENSOR_PLACEMENT_OFFSET;
            ams_config.pole_pairs = 2;

            ams_service(pRotarySensor, ams_config, i_ams);

//            ams_sensor_server(i_ams, NUM_OF_AMS_INTERFACES, pRotarySensor);

  //          ams_rotary_sensor_direct_method(pRotarySensor, AMS_INIT_SETTINGS1, AMS_INIT_SETTINGS2, 2555);


        }

    }

    return 0;
}

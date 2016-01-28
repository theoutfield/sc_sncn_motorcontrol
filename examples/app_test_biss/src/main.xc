/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//BiSS libs
#include <biss_service.h>


/* Test BiSS Encoder Client */
void biss_test(client interface BISSInterface i_biss) {
    timer t;
    unsigned int start_time, end_time;
    int count = 0;
    int real_count = 0;
    int velocity = 0;
    unsigned int position = 0;
    unsigned int status = 0;

    while(1) {
        t :> start_time;

        /* get position from BiSS Encoder */
        { count, position, status } = i_biss.get_biss_position();
        t :> end_time;
        { real_count, void, void } = i_biss.get_biss_real_position();

        /* get velocity from BiSS Encoder */
        velocity = i_biss.get_biss_velocity();

        xscope_int(COUNT, count);                           //absolute count
        xscope_int(REAL_COUNT, real_count);                 //real internal absolute count
        xscope_int(POSITION, position);                     //singleturn position
        xscope_int(VELOCITY, velocity);                     //velocity in rpm
        xscope_int(ERROR_BIT, (status&0b10) * 500);         //error bit, should be 0
        xscope_int(WARNING_BIT, (status&0b01) * 1000);      //warning bit, should be 0
        xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds

        delay_milliseconds(1);
    }
}

BISSPorts biss_ports = {QEI_PORT, SOMANET_IFM_GPIO_D0, IFM_TILE_CLOCK_2};

int main() {
    interface BISSInterface i_biss[5]; //array of interfaces for biss server

    par {
        /* Test BiSS Encoder Client */
        on tile[COM_TILE]: biss_test(i_biss[0]);


        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
            /* BiSS server */
            {
                BISSConfig biss_config;
                biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                biss_config.status_length = BISS_STATUS_LENGTH;
                biss_config.crc_poly = BISS_CRC_POLY;
                biss_config.pole_pairs = 2;
                biss_config.polarity = BISS_POLARITY;
                biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                biss_config.timeout = BISS_TIMEOUT;
                biss_config.max_ticks = BISS_MAX_TICKS;
                biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                biss_service(biss_ports, biss_config, i_biss);
            }
    }
    return 0;
}

/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position information
 * @author Synapticon GmbH <support@synapticon.com>
 */


#include <xscope.h>
#include <timer.h>
#include <biss_server.h>
#include <biss_client.h>
#include <refclk.h>


on tile[IFM_TILE]: clock clk_biss = XS1_CLKBLK_1 ;
port out p_ifm_biss_clk = GPIO_D0;

/* Test BiSS Encoder Client */
void biss_test(client interface i_biss i_biss) {
    timer t;
    unsigned int start_time, end_time;
    int count = 0;
    int real_count = 0;
    int velocity = 0;
    unsigned int position = 0;
    unsigned int status = 0;

    i_biss.set_count(0);

    while(1) {
        t :> start_time;

        /* get position from BiSS Encoder */
        { count, position, status } = i_biss.get_position();
        t :> end_time;
        { real_count, void, void } = i_biss.get_real_position();

        /* get velocity from BiSS Encoder */
        velocity = i_biss.get_velocity();

        xscope_int(COUNT, count);                           //absolute count
        xscope_int(REAL_COUNT, real_count);                 //real internal absolute count
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);
        xscope_int(ERROR_BIT, (status&0b10) * 500);         //error bit, should be 0
        xscope_int(WARNING_BIT, (status&0b01) * 1000);      //warning bit, should be 0
        xscope_int(TIME, (end_time-start_time)/USEC_STD);    //time to get the data in microseconds

        delay_milliseconds(1);
    }
}


int main() {
    interface i_biss i_biss[1]; //array of interfaces for biss server

    par {
        /* Test BiSS Encoder Client */
        on tile[COM_TILE]: biss_test(i_biss[0]);


        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: {
            /* BiSS server */
            {
                biss_par biss_params;
                run_biss(i_biss, 1, p_ifm_biss_clk, p_ifm_encoder, clk_biss, biss_params, 2);
            }
        }
    }
    return 0;
}

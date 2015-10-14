#include <CORE_C22-rev-a.inc>
#include <IFM_A1-rev-a.inc>

/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position information
 * @author Synapticon GmbH <support@synapticon.com>
 */


#include <xscope.h>
#include <timer.h>
#include <biss_server.h>


/* Test BiSS Encoder Client */
void biss_test(client interface i_biss i_biss) {
    timer t;
    unsigned int start_time, end_time;
    int count = 0;
    unsigned int position = 0;
    unsigned int status = 0;

    while(1) {
        t :> start_time;

        /* get position from BiSS Encoderr */
        { count, position, status } = i_biss.get_position();

        t :> end_time;
        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ERROR, (status&0b10) * 500);         //error bit, should be 0
        xscope_int(WARNING, (status&0b01) * 1000);      //warning bit, should be 0
        xscope_int(TIME, (end_time-start_time)/100);    //time to get the data in microseconds
        delay_milliseconds(1);
    }
}

int main() {
    interface i_biss i_biss[2]; //array of interfaces for biss server

    par {
        /* Test BiSS Encoder Client */
        on tile[COM_TILE]: biss_test(i_biss[0]);


        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: {
            /* BiSS server */
            biss_par biss_params;

#if(BISS_DATA_PORT == ENC_CH1)
            run_biss(i_biss, qei_q_ch1, p_ifm_encoder_ch1, clk_biss, 10, 1, biss_params, 2);
#else
            p_ifm_encoder_ch1 <: 0b1000;
            run_biss(i_biss, qei_q_ch1, p_ifm_encoder_ch2, clk_biss, 10, 1, biss_params, 2);
        }
#endif
    }
    return 0;
}

/*
 * app_test_biss.xc
 *
 *  Created on: Oct 9, 2015
 *      Author: romuald
 */
#include <IFM-rev-a.inc>

#include <xscope.h>
#include <timer.h>
#include <biss_server.h>


void test_biss(client interface i_biss i_biss) {
    timer t;
    while(1) {
        int multiturn, singleturn;
        unsigned int status, start_time, end_time;
        t :> start_time;
        { multiturn, singleturn, status } = i_biss.position();
        t :> end_time;
        xscope_int(MULTITURN, multiturn);
        xscope_int(SINGLETURN, singleturn);
        xscope_int(ERROR, (status&0b10) * 500);
        xscope_int(WARNING, (status&0b01) * 1000);
        xscope_int(TIME, (end_time-start_time)/100);
        delay_milliseconds(1);
    }
}

int main() {
    interface i_biss i_biss[2];

    par {
        on tile[0]: test_biss(i_biss[0]);

        on tile[3]: {
            biss_par biss_params;
            init_biss_param(biss_params, 28, 12, 13, 2, 0b110000);

#if(P_BISS_DATA == ENC_CH1)
            run_biss(i_biss, qei_q_ch1, p_ifm_encoder_ch1, clk_biss, 10, 1, biss_params, 2);
#else
            p_ifm_encoder_ch1 <: 0b1000;
            run_biss(i_biss, qei_q_ch1, p_ifm_encoder_ch2, clk_biss, 10, 1, biss_params, 2);
        }
#endif
    }
    return 0;
}

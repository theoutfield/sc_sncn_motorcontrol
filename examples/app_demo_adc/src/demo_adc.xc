/*
 * demo_adc.xc
 *
 *  Created on: Jul 13, 2017
 *      Author: Synapticon GmbH
 */

#include <demo_adc.h>


void demo_adc(interface ADCInterface client i_adc)
{
    timer t;
    unsigned time=0;


    int a1=0, a2=0, a3=0, a4=0, a5=0, a6=0;
    int b1=0, b2=0, b3=0, b4=0, b5=0, b6=0;

    int a1a2=0, a3a4=0, a5a6=0;
    int b1b2=0, b3b4=0, b5b6=0;

    int period=10000;

    while(i_adc.status()!=ACTIVE);

    t :> time;
    while (1)
    {
        i_adc.set_channel(SGL_A1_B1);
        {a1, b1} = i_adc.sample_and_send();

        i_adc.set_channel(SGL_A2_B2);
        {a2, b2} = i_adc.sample_and_send();

        i_adc.set_channel(SGL_A3_B3);
        {a3, b3} = i_adc.sample_and_send();

        i_adc.set_channel(SGL_A4_B4);
        {a4, b4} = i_adc.sample_and_send();

        i_adc.set_channel(SGL_A5_B5);
        {a5, b5} = i_adc.sample_and_send();

        i_adc.set_channel(SGL_A6_B6);
        {a6, b6} = i_adc.sample_and_send();

        i_adc.set_channel(DIFF_A1A2_B1B2);
        {a1a2, b1b2} = i_adc.sample_and_send();

        i_adc.set_channel(DIFF_A3A4_B3B4);
        {a3a4, b3b4} = i_adc.sample_and_send();

        i_adc.set_channel(DIFF_A5A6_B5B6);
        {a5a6, b5b6} = i_adc.sample_and_send();

        xscope_int(A1, a1);
        xscope_int(B1, b1);
        xscope_int(A2, a2);
        xscope_int(B2, b2);
        xscope_int(A3, a3);
        xscope_int(B3, b3);
        xscope_int(A4, a4);
        xscope_int(B4, b4);
        xscope_int(A5, a5);
        xscope_int(B5, b5);
        xscope_int(A6, a6);
        xscope_int(B6, b6);
        xscope_int(A1A2, a1a2);
        xscope_int(B1B2, b1b2);
        xscope_int(A3A4, a3a4);
        xscope_int(B3B4, b3b4);
        xscope_int(A5A6, a5a6);
        xscope_int(B5B6, b5b6);

    } // while(1)

}



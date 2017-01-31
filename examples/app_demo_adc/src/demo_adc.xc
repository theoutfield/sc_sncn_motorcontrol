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

        xscope_int(A1, a1);
        xscope_int(B1, b1);

        xscope_int(A2, a2);
        xscope_int(B2, b2);

    } // while(1)

}



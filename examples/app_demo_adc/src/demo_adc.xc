/*
 * demo_adc.xc
 *
 *  Created on: Jul 13, 2017
 *      Author: Synapticon GmbH
 */

#include <demo_adc.h>


void demo_ad7265(interface ADCInterface client i_adc)
{
    timer t;
    unsigned time=0;


    int a0=0, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0;
    int b0=0, b1=0, b2=0, b3=0, b4=0, b5=0, b6=0;

    int a1a2=0, a3a4=0, a5a6=0;
    int b1b2=0, b3b4=0, b5b6=0;

    int period=10000;

    while(i_adc.status()!=ACTIVE);

    t :> time;
    while (1)
    {
        {a0, b0} = i_adc.get_channel(AD7265_SGL_A1_B1);
        {a1, b1} = i_adc.get_channel(AD7265_SGL_A2_B2);
        {a2, b2} = i_adc.get_channel(AD7265_SGL_A3_B3);
        {a3, b3} = i_adc.get_channel(AD7265_SGL_A4_B4);
        {a4, b4} =  i_adc.get_channel(AD7265_SGL_A5_B5);
        {a5, b5} = i_adc.get_channel(AD7265_SGL_A6_B6);
        {a1a2, b1b2} = i_adc.get_channel(AD7265_DIFF_A1A2_B1B2);
        {a3a4, b3b4} = i_adc.get_channel(AD7265_DIFF_A3A4_B3B4);
        {a5a6, b5b6} = i_adc.get_channel(AD7265_DIFF_A5A6_B5B6);

        xscope_int(A0, a0);
        xscope_int(B0, b0);
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
        xscope_int(A1A2, a1a2);
        xscope_int(B1B2, b1b2);
        xscope_int(A3A4, a3a4);
        xscope_int(B3B4, b3b4);
        xscope_int(A5A6, a5a6);
        xscope_int(B5B6, b5b6);

        delay_milliseconds(2);

    } // while(1)

}

void demo_ad7949(interface ADCInterface client i_adc)
{
    timer t;
    unsigned time=0;

    int a0=0, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, a7=0;
    int b0=0, b1=0, b2=0, b3=0, b4=0, b5=0, b6=0, b7=0;

    while(i_adc.status()!=ACTIVE);

    t :> time;
    while (1)
    {
        {a0, b0} = i_adc.get_channel(AD7949_CHANNEL_0);
        {a1, b1} = i_adc.get_channel(AD7949_CHANNEL_1);
        {a2, b2} = i_adc.get_channel(AD7949_CHANNEL_2);
        {a3, b3} = i_adc.get_channel(AD7949_CHANNEL_3);
        {a4, b4} = i_adc.get_channel(AD7949_CHANNEL_4);
        {a5, b5} = i_adc.get_channel(AD7949_CHANNEL_5);
        {a6, b6} = i_adc.get_channel(AD7949_CHANNEL_6);
        {a7, b7} = i_adc.get_channel(AD7949_CHANNEL_7);

        xscope_int(A0, a0);
        xscope_int(B0, b0);
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
        xscope_int(A7, a7);
        xscope_int(B7, b7);

        delay_milliseconds(2);

    } // while(1)

}

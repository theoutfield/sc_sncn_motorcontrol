/**
 * @file filters_lib.xc
 * @brief Filters Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <filters.h>
#include <stdio.h>


/* initialization for fixed length digital filter (moving average filter) */
void init_filter(int filter_buffer[], int &index, int filter_length)
{
    int i;
    for (i=0; i<filter_length; i++) {
        filter_buffer[i] = 0;
    }
    index = 0;
}

/* fixed length digital filter (moving average filter) */
int filter(int filter_buffer[], int &index, int filter_length, int input)
{
    int filter_output = 0;
    filter_buffer[index] = input;
    index = (index+1)%(filter_length);

    for (int i=0; i<filter_length; i++) {
        int mod = (index - 1 - i);
        if (mod < 0) {
            mod += filter_length;
        }
        filter_output += filter_buffer[mod];
    }
    filter_output = filter_output/ filter_length;
    return filter_output;
}


int _modified_internal_filter(int filter_buffer[], int &index, int filter_length, int input)
{
    int i, j = 0, mod, filter_output =0;
    filter_buffer[index] = input;
    index = (index+1)%(filter_length);

    for (i=0; i<filter_length; i++) {
        mod = (index - 1 - j)%filter_length;
        if (mod<0) {
            mod = filter_length + mod;
        }
        filter_output += filter_buffer[mod];
        j++;
    }
    return filter_output;
}



/**
 * @brief setting the cut-off frequency and intializing the parameters of the first-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void first_order_LP_filter_init(int f_c, int T_s, FirstOrderLPfilterParam &param )
{
    double f_c_max=0.00, omega_T=0.00;

    param.y_k  =0.00;
    param.y_k_1=0.00;

    param.T_s = T_s;
    omega_T = (6.28318530718 * ((double)f_c) * ((double)T_s))/1000000.00;
    param.a1 = 1.00/(1.00+omega_T);
    param.b0 = omega_T/(1.00+omega_T);
}


/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 9us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double first_order_LP_filter_update(double *x_k, FirstOrderLPfilterParam &param)
{
    param.y_k = (param.a1 * (param.y_k_1)) + (param.b0 * (*x_k));
    param.y_k_1 = (param.y_k);

    return param.y_k;
}



/**
 * @brief setting the cut-off frequency and intializing the parameters of the second-order-LP-filters.
 * @time needed to be processed: 14us when the 100MHz teil is fully loaded.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void second_order_LP_filter_init(int f_c, int T_s, SecondOrderLPfilterParam &param )
{

    double fs=0.00, w=0.00, z=0.00, d=0.00;

    param.y_k  =0.00;
    param.y_k_1=0.00;
    param.y_k_2=0.00;

    fs= 1000000.00/((double)(T_s));
    w = 6.28318530718 * ((double)f_c);
    z = 0.60;

    d = (fs*fs) + (2*z*w*fs) + (w*w);

    param.T_s = T_s;

    param.a1 = (2*fs*fs) + (2*z*w*fs);
    param.a1/= d;

    param.a2 = -(fs*fs);
    param.a2/= d;

    param.b0 = w*w;
    param.b0/= d;
}

/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 20us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double second_order_LP_filter_update(double *x_k, SecondOrderLPfilterParam &param)
{
    param.y_k = (param.a1 * (param.y_k_1)) + (param.a2 * (param.y_k_2)) + (param.b0 * (*x_k));
    param.y_k_2 = (param.y_k_1);
    param.y_k_1 = (param.y_k);

    return param.y_k;
}

/**
 * @brief setting the cut-off frequency and intializing the parameters of the third-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void third_order_LP_filter_init(int f_c, int T_s, ThirdOrderLPfilterParam &param )
{
    double fs=0.00, w=0.00, z=0.00, d=0.00;

    param.y_k  =0.00;
    param.y_k_1=0.00;
    param.y_k_2=0.00;
    param.y_k_3=0.00;

    fs= 1000000.00/((double)(T_s));
    w = 6.28318530718 * ((double)f_c);
    z = 0.40;

    d = (fs*fs*fs) + (fs*fs*w) + (fs*fs*2*z*w) + (fs*w*w) + (fs*2*z*w*w) + (w*w*w);

    param.T_s = T_s;


    param.a1 =  (3*fs*fs*fs) + (2*w*fs*fs) + (4*z*w*fs*fs) + (w*w*fs) + (2*z*w*w*fs);
    param.a1/=  d;

    param.a2 = (-1)*((3*fs*fs*fs) + (w*fs*fs) + (2*z*w*fs*fs));
    param.a2/=  d;

    param.a3 = fs*fs*fs;
    param.a3/=  d;

    param.b0 = w*w*w;
    param.b0/=  d;
}

/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: ?? us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double third_order_LP_filter_update(double *x_k, ThirdOrderLPfilterParam &param)
{
    param.y_k = (param.a1 * (param.y_k_1)) + (param.a2 * (param.y_k_2)) + (param.a3 * (param.y_k_3)) + (param.b0 * (*x_k));
    param.y_k_3 = param.y_k_2;
    param.y_k_2 = param.y_k_1;
    param.y_k_1 = param.y_k;

    return param.y_k;
}

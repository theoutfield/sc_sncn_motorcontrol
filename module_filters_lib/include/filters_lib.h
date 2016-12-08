/**
 * @file filters_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once


#define US_DENOMINATOR          1000000

/**
 * @brief Structure type to set the parameters of the first-order-LP-filters.
 */
typedef struct
{
    int T_s;    //Sampling-Time in microseconds
    double a1;
    double b0;
    double y_k;
    double y_k_1;
} FirstOrderLPfilterParam;


/**
 * @brief Structure type to set the parameters of the second-order-LP-filters.
 */
typedef struct
{
    int T_s;    //Sampling-Time in microseconds
    double a1;
    double a2;
    double b0;
    double y_k;
    double y_k_1;
    double y_k_2;
} SecondOrderLPfilterParam;


/**
 * @brief Structure type to set the parameters of the third-order-LP-filters.
 */
typedef struct
{
    int T_s;    //Sampling-Time in microseconds
    double a1;
    double a2;
    double a3;
    double b0;
    double y_k;
    double y_k_1;
    double y_k_2;
    double y_k_3;
} ThirdOrderLPfilterParam;


/**
 * @brief setting the cut-off frequency and intializing the parameters of the first-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void first_order_LP_filter_init(int f_c, int T_s_considered, FirstOrderLPfilterParam &param );

/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 9us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double first_order_LP_filter_update(double *x_k, FirstOrderLPfilterParam &param);

/**
 * @brief setting the cut-off frequency and intializing the parameters of the second-order-LP-filters.
 * @time needed to be processed: 14us when the 100MHz teil is fully loaded.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void second_order_LP_filter_init(int f_c, int T_s_considered, SecondOrderLPfilterParam &param );

/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 20us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double second_order_LP_filter_update(double *x_k, SecondOrderLPfilterParam &param);

/**
 * @brief setting the cut-off frequency and intializing the parameters of the third-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 */
void third_order_LP_filter_init(int f_c, int T_s_considered, ThirdOrderLPfilterParam &param );


/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: ?? us when the 100MHz teil is fully loaded.
 *
 * @param output ->  the filtered signal.
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 */
double third_order_LP_filter_update(double *x_k, ThirdOrderLPfilterParam &param);




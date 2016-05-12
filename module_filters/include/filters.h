/**
 * @file filters.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once


#define US_DENOMINATOR          1000000


/**
 * @brief Structure type to set the parameters of the first-order-LP-filters.
 */
typedef struct {
    int T_s;    //Sampling-Time in microseconds
    float a1;
    float b0;
} FirstOrderLPfilterParam;


/**
 * @brief Structure type to set the parameters of the second-order-LP-filters.
 */
typedef struct {
    int T_s;    //Sampling-Time in microseconds
    float a1;
    float a2;
    float b0;
} SecondOrderLPfilterParam;


/**
 * @brief Structure type to set the parameters of the third-order-LP-filters.
 */
typedef struct {
    int T_s;    //Sampling-Time in microseconds
    float a1;
    float a2;
    float a3;
    float b0;
} ThirdOrderLPfilterParam;


/**
 * @brief setting the cut-off frequency and intializing the parameters of the first-order-LP-filters.
 *
 * @param cut-off frequency in Hz.
 * @param sampling-time in us (microseconds).
 * @param filter parameters.
 */
void first_order_LP_filter_init(int f_c, int T_s_considered, FirstOrderLPfilterParam &param );


/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 9us when the 100MHz teil is fully loaded.
 *
 * @param output, the filtered signal.
 * @param buffer, the previous output.
 * @param input, the input signal.
 * @param sample-time in us (microseconds).
 * @param filter parameters.
 */
void first_order_LP_filter_update(float *y_k, float *y_k_1n, float *x_k, int T_s, FirstOrderLPfilterParam &param);

void first_order_LP_filter_shift_buffers(float *y_k, float *y_k_1n);


/**
 * @brief setting the cut-off frequency and intializing the parameters of the second-order-LP-filters.
 * @time needed to be processed: 14us when the 100MHz teil is fully loaded.
 *
 * @param cut-off frequency in Hz.
 * @param sampling-time in us (microseconds).
 * @param filter parameters.
 */
void second_order_LP_filter_init(int f_c, int T_s_considered, SecondOrderLPfilterParam &param );


/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: 20us when the 100MHz teil is fully loaded.
 *
 * @param output, the filtered signal.
 * @param buffer, the previous output.
 * @param buffer, the previous previous output.
 * @param input, the input signal.
 * @param sample-time in us (microseconds).
 * @param filter parameters.
 */
void second_order_LP_filter_update(float *y_k, float *y_k_1n, float *y_k_2n, float *x_k, int T_s, SecondOrderLPfilterParam &param);


void second_order_LP_filter_shift_buffers(float *y_k, float *y_k_1n, float *y_k_2n);


/**
 * @brief setting the cut-off frequency and intializing the parameters of the second-order-LP-filters.
 *
 * @param cut-off frequency in Hz.
 * @param sampling-time in us (microseconds).
 * @param filter parameters.
 */
void third_order_LP_filter_init(int f_c, int T_s_considered, ThirdOrderLPfilterParam &param );


/**
 * @brief filtering the signal x_k.
 * @time needed to be processed: ?? us when the 100MHz teil is fully loaded.
 *
 * @param output, the filtered signal.
 * @param buffer, the previous output.
 * @param buffer, the previous previous output.
 * @param buffer, the previous previous previous output.
 * @param input, the input signal.
 * @param sample-time in us (microseconds).
 * @param filter parameters.
 */
void third_order_LP_filter_update(float *y_k, float *y_k_1n, float *y_k_2n, float *y_k_3n, float *x_k, int T_s, ThirdOrderLPfilterParam &param);


void third_order_LP_filter_shift_buffers(float *y_k, float *y_k_1n, float *y_k_2n, float *y_k_3n);






/**
 * @file filters.h
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
 * @brief Initialize Moving Average Filter Configuration.
 *
 * @param filter_buffer Reference to the samples array to initialize.
 * @param index Reference to the index variable to initialize.
 * @param filter_length Defines the length of the filter.
 *
 *@return void
 */
void init_filter(int filter_buffer[], int &index, int filter_length);

/**
 * @brief Get moving average filtered output.
 *
 * @param filter_buffer Samples to filter.
 * @param index Index of the filter.
 * @param filter_length Defines the length of the filter.
 * @param input New sample.
 *
 * @return Filtered output.
 */
int filter(int filter_buffer[], int & index, int filter_length, int input);

/**
 * @brief Intializing the parameters of the first-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure
 *
 * @return void
 */
void first_order_LP_filter_init(int f_c, int T_s, FirstOrderLPfilterParam &param );

/**
 * @brief filtering the signal x_k.
 *
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 *
 * @return       ->  filtered value
 */
double first_order_LP_filter_update(double *x_k, FirstOrderLPfilterParam &param);

/**
 * @brief Intializing the parameters of the second-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure.
 *
 * @return      -> filtered value
 */
void second_order_LP_filter_init(int f_c, int T_s, SecondOrderLPfilterParam &param);

/**
 * @brief filtering the signal x_k.
 *
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 *
 * @return       ->  filtered value
 */
double second_order_LP_filter_update(double *x_k, SecondOrderLPfilterParam &param);

/**
 * @brief Intializing the parameters of the third-order-LP-filters.
 *
 * @param f_c   -> cut-off frequency in Hz.
 * @param T_s   -> sampling-time in us (microseconds).
 * @param param -> filter parameters structure.
 *
 * @return      -> filtered value
 */
void third_order_LP_filter_init(int f_c, int T_s, ThirdOrderLPfilterParam &param);

/**
 * @brief filtering the signal x_k.
 *
 * @param x_k    ->  the input signal.
 * @param param  ->  filter parameters.
 *
 * @return       ->  filtered value
 */
double third_order_LP_filter_update(double *x_k, ThirdOrderLPfilterParam &param);






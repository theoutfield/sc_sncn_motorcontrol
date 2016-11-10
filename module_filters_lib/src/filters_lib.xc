/**
 * @file filters_lib.xc
 * @brief Filters Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <filters_lib.h>
#include <stdio.h>


void first_order_LP_filter_init(int f_c, int T_s_considered, FirstOrderLPfilterParam &param )
{
    float f_c_max, omega_T;
    param.T_s = T_s_considered;
    f_c_max = US_DENOMINATOR/(6.28318530718 * ((float)T_s_considered));
    if (f_c < 0){
        f_c = 0;
    }else if (f_c > ((int)f_c_max)){
        f_c = (int)f_c_max;
        printf("\nERROR: The cutt-off frequency of the first_order_LP_filter is higher than the limit.\nMax f_c = %.2f\n",f_c_max);
    }
    omega_T = (6.28318530718 * ((float)f_c) * ((float)T_s_considered)) / US_DENOMINATOR;
    param.a1 = (1 - omega_T);
    param.b0 = omega_T;
}


void first_order_LP_filter_update(float *y_k, float *y_k_1n, float *x_k, int T_s, FirstOrderLPfilterParam &param)
{
    if( (T_s > (2 * param.T_s)) || (T_s < (param.T_s / 2))){
        printf("\nERROR in first_order_LP_filetr: The initialized T_s is more than 100percent bigger/smaller than the loop-time\n T_s=%d Loop-Time=%d",param.T_s, T_s);
    }
    *y_k = (param.a1 * (*y_k_1n)) + (param.b0 * (*x_k));
}


void first_order_LP_filter_shift_buffers(float *y_k, float *y_k_1n)
{
    *y_k_1n = (*y_k);
}


void second_order_LP_filter_init(int f_c, int T_s_considered, SecondOrderLPfilterParam &param )
{
    float f_c_max, omega_T;
    param.T_s = T_s_considered;
    f_c_max = US_DENOMINATOR/(6.28318530718 * ((float)T_s_considered));
    if (f_c < 0){
        f_c = 0;
    }else if (f_c > ((int)f_c_max)){
        f_c = (int)f_c_max;
        printf("\nERROR: The cutt-off frequency of the second_order_LP_filter is higher than the limit.\nMax f_c = %.2f\n",f_c_max);
    }
    omega_T = (6.28318530718 * ((float)f_c) * ((float)T_s_considered)) / US_DENOMINATOR;
    param.a1 = 2 * (1 - omega_T);
    param.a2 = -(1 - omega_T) * (1 - omega_T);
    param.b0 = omega_T * omega_T;
}


void second_order_LP_filter_update(double *y_k, double *y_k_1n, double *y_k_2n, double *x_k, int T_s, SecondOrderLPfilterParam &param)
{
    if( (T_s > (2 * param.T_s)) || (T_s < (param.T_s / 2))){
        printf("\nERROR in second_order_LP_filetr: The initialized T_s is more than 100percent bigger/smaller than the loop-time\n T_s=%d Loop-Time=%d",param.T_s, T_s);
    }
    *y_k = (param.a1 * (*y_k_1n)) + (param.a2 * (*y_k_2n)) + (param.b0 * (*x_k));
}


void second_order_LP_filter_shift_buffers(double *y_k, double *y_k_1n, double *y_k_2n)
{
    *y_k_2n = (*y_k_1n);
    *y_k_1n = (*y_k);
}


void third_order_LP_filter_init(int f_c, int T_s_considered, ThirdOrderLPfilterParam &param )
{
    float f_c_max, omega_T;
    param.T_s = T_s_considered;
    f_c_max = US_DENOMINATOR/(6.28318530718 * ((float)T_s_considered));
    if (f_c < 0){
        f_c = 0;
    }else if (f_c > ((int)f_c_max)){
        f_c = (int)f_c_max;
        printf("\nERROR: The cutt-off frequency of the second_order_LP_filter is higher than the limit.\nMax f_c = %.2f\n",f_c_max);
    }
    omega_T = (6.28318530718 * ((float)f_c) * ((float)T_s_considered)) / US_DENOMINATOR;
    param.a1 = 3 * (1 - omega_T);
    param.a2 = -3 * (1 - omega_T) * (1 - omega_T);
    param.a3 = (1 - omega_T) * (1 - omega_T) * (1 - omega_T);
    param.b0 = omega_T * omega_T * omega_T;
}


void third_order_LP_filter_update(float *y_k, float *y_k_1n, float *y_k_2n, float *y_k_3n, float *x_k, int T_s, ThirdOrderLPfilterParam &param)
{
    if( (T_s > (2 * param.T_s)) || (T_s < (param.T_s / 2))){
        printf("\nERROR in third_order_LP_filetr: The initialized T_s is more than 100percent bigger/smaller than the loop-time\n T_s=%d Loop-Time=%d",param.T_s, T_s);
    }
    *y_k = (param.a1 * (*y_k_1n)) + (param.a2 * (*y_k_2n)) + (param.a3 * (*y_k_3n)) + (param.b0 * (*x_k));
}


void third_order_LP_filter_shift_buffers(float *y_k, float *y_k_1n, float *y_k_2n, float *y_k_3n)
{
    *y_k_3n = (*y_k_2n);
    *y_k_2n = (*y_k_1n);
    *y_k_1n = (*y_k);
}























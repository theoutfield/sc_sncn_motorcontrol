#pragma once



int check_tolerance(int actual_value, int compare_value);


/*=== calculate six times the mean value over one periode ====
 *
 *
 */
int     calc_mean_one_periode (int iCX[], int iXX[], int new_value, int old_value, int speed_clock , int index_array);




int     low_pass_pt1_filter(int filter_sum[], int iFilterIndex, int iFactor, int pt1_new_value);





{int, int} cartesian_to_polar_conversion(int xValue, int yValue );



int     check_limits(int input, int max_limit);

int     calc_hysteresis_and_diff_limit(int input, int reference, int hys_percent, int limit_percent);

int     calc_hysteresis_and_limit(int input, int hys_value, int max_value);


//----------------------------------- test purpose -------------------

//void calculate_diff_angle(foc_variables_t& fp);



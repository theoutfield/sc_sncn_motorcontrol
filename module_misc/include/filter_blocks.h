/**
 * @file filter_blocks.h
 * @brief Moving Average Filter Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**
 * @brief Initialize Moving Average Filter Configuration.
 *
 * @param filter_buffer Reference to the samples array to initialize.
 * @param index Reference to the index variable to initialize.
 * @param filter_length Defines the length of the filter.
 *
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


/* Internal function */
int _modified_internal_filter(int filter_buffer[], int & index, int filter_length, int input);


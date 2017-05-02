/*
 * data_processing.h
 *
 *  Created on: Mar 30, 2017
 *      Author: jgofre
 */


#ifndef DATA_PROCESSING_H_
#define DATA_PROCESSING_H_

//#include <torque_ripple_correction.h>
#include <user_config.h>

#define NB_OF_MEASUREMENTS 600
#define MAX_COLUMNS 2
#define MAX_ROWS 90

#define WRITE_BUFFER_SIZE MAX_COLUMNS*11+22
#define READ_BUFFER_SIZE WRITE_BUFFER_SIZE*MAX_ROWS

void read_csv_file_to_array(int Numbers_array[MAX_COLUMNS][MAX_ROWS]);

// Function that reads a 2D array of dimensions real_column_number x real_row_number
//  Inputs : array => 2-D array of integers of dimension MAX_COLUMNS x NB_OF_MEASUREMENT_POINTS
//           real_row_number => int : real number of rows in the array
//           name => string : name of the file in which the array will be written the path can be added at the beginning of the name
//           headers => string : first row of the array, title of each column
//
//  Examples : name = "/Documents/array.csv"
//             headers = "Time, Current, Voltage"
void write_array_to_csv_file(int array[MAX_COLUMNS][MAX_ROWS],int real_row_number, int real_column_number, char * name, char* headers);

#endif /* DATA_PROCESSING_H_ */

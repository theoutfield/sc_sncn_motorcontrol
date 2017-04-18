/*
 * data_processing.xc
 *
 *  Created on: Mar 30, 2017
 *      Author: jgofre
 */


#include <data_processing.h>
#include <syscall.h>
#include <print.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


unsigned char readBuffer[READ_BUFFER_SIZE];


void write_array_to_csv_file(int array[MAX_COLUMNS][MAX_ROWS],int real_row_number, int real_column_number, char * name, char* headers)
{

    char buffer [WRITE_BUFFER_SIZE];

    strcpy(buffer, headers);
    strcat(buffer, "\n");

    //Create the file
    int fd = _open(name, O_WRONLY | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);
    printf("Trying to write \n");
    _write(fd, buffer, strlen(buffer));
    if (_close(fd) != 0) {
        printf("Error: _close failed.\n");
        exit(1);
    }

    // Open the file and write line after line
    fd = _open(name, O_WRONLY | O_APPEND, S_IREAD | S_IWRITE);
    if (fd == -1) {
        printstrln("Error: _open failed");
        exit(1);
    }

    for (int row= 0; row < real_row_number; row++)
    {
        strcpy(buffer, "");
        for (int column = 0; column < real_column_number; column++)
        {
            int value = array[column][row];

            char string[8+3];
            if (column != real_column_number-1)
            {
                sprintf(string, "%d, ", value);
            }
            else{
                sprintf(string, "%d \n", value);
            }
            strcat(buffer, string);
        }
        _write(fd, buffer, strlen(buffer));

    }
    // Close File
    if (_close(fd) != 0) {
        printf("Error: _close failed.\n");
        exit(1);
    }
    else printf("File written\n");
}


void read_csv_file_to_array(int Numbers_array[MAX_COLUMNS][MAX_ROWS])
{
    //Open the file

    int fd = _open("Data/Test_autotuning2_10_generations_10_competitors.csv", O_RDONLY, 0);
    if (fd == -1) {
      printstrln("Error: _open failed");
      exit(1);
    }
    printf("opening file...\n");

    //Store the content of the file in the buffer
    _read(fd, readBuffer, READ_BUFFER_SIZE);

    int row = 0;
    int column = 0;
    int value = 0;
    int sign = 1;
    int new_row =0;
    int new_column =0;
    int stop_value = 0;
    int is_not_value = 0;
    int index = 0;
    char eof = ',';
    char eol = '\n';

    //Parse the buffer from csv standard
    while (sizeof(readBuffer) > index && row < 10)
    {

        while (!new_row && !new_column)
        {
            char c = readBuffer[index];
            index++;
            if(isdigit(c)>0)
            {
                if (!stop_value)
                {
                    value *= 10;
                    value += c - '0';
                }
            }
            else if (c == '-')
            {
                sign = -1;
            }
            else if (c == eof)
            {
                new_column = 1;
            }
            else if (c == eol)
            {
                new_row = 1;
            }
            else if (c == '.')
            {
                stop_value = 1;
            }
            else if (c != ' ')
            {
                is_not_value = 1;
            }

        }
        stop_value = 0;
        value *= sign;
        //printf("Numbers array[%d][%d] : %d\n Value : %d\n", column, row, value, is_not_value);
        Numbers_array[column][row] = value;
        value = 0;
        sign = 1;
        if (new_row)
        {
            new_row = 0;
            if(is_not_value)
            {
                is_not_value = 0;
            }
            else
            {
                row++;
                column = 0;
            }


        }
        if (new_column)
        {
            new_column = 0;
            if(is_not_value)
            {
                is_not_value = 0;
            }
            else column++;
        }
    }

    if (_close(fd) != 0) {
      printstrln("Error: _close failed.");
      exit(1);
    }else printstrln("File closed.");
}

void write_array_to_csv_file2(int array[MAX_COLUMNS][MAX_ROWS],int nb_of_steps, char * name, char* headers)
{
    //Open the file
    //int count_bytes_written = 0;
    printf("Opening file : %s ; %d NB OF STEPS\n", name, nb_of_steps);
    int fd = _open(name, O_WRONLY | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);
    if (fd == -1) {
      printstrln("Error: _open failed");
      exit(1);
    }

    char buffer [WRITE_BUFFER_SIZE];

    strcpy(buffer, headers);
    //count_bytes_written += sizeof(headers);
    strcat(buffer, "\n");
    //count_bytes_written ++;
    for (int row= 0; row < nb_of_steps; row++)
    {
        for (int column = 0; column < MAX_COLUMNS; column++)
        {
            int value = array[column][row];

            char string[8+4];
            if (column != MAX_COLUMNS-1)
            {
                sprintf(string, "%d , ", value);
            }
            else{
                sprintf(string, "%d  \n", value);
            }
            strcat(buffer, string);
           // count_bytes_written += 9;
        }
    }
    printf("Trying to write \n");
    _write(fd, buffer, strlen(buffer));
    if (_close(fd) != 0) {
            printf("Error: _close failed.\n");
            exit(1);
        }
    //

}


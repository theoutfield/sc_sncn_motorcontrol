/* fixed length digital filter (moving average filter)*/

/**
 * \brief Initialise Moving average Filter Parameters
 *
 *  Input
 * \param filter_buffer
 * \param index
 * \param filter_length defines the length of the filter
 *
 */
void init_filter(int filter_buffer[], int &index, int filter_length);

/**
 * \brief Get moving average filtered output by passing the sampled data to be filtered as input
 *
 *  Input
 * \param filter_buffer
 * \param index
 * \param filter_length defines the length of the filter
 * \param input data sampled at fixed time
 *
 * Output
 * \return filtered output
 */
int filter(int filter_buffer[], int &index, int filter_length, int input);

/*Internal function*/
int _modified_internal_filter(int filter_buffer[], int &index, int filter_length, int input);

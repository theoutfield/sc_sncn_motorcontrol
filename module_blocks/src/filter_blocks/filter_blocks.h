/* fixed length digital filter (moving average filter)*/
void init_filter(int filter_buffer[], int &index, int filter_length);
int filter(int filter_buffer[], int &index, int filter_length, int input);

int _modified_internal_filter(int filter_buffer[], int &index, int filter_length, int input);

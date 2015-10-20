/**
 * @file biss_client.xc
 * @brief BiSS Encoder Client Functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_client.h>

{ int, unsigned int, unsigned int } get_biss_position(client interface i_biss i_biss) {
    int count;
    unsigned int position, status;
    { count, position, status } = i_biss.get_position();
    return { count, position, status };
}

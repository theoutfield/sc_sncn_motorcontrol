/**
 * @file arctan.xc
 * @brief Arc tangent calculations
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <stdint.h>

extern uint16_t sine_third[];
extern uint16_t sine_table[];

/* for sin(x) and every function with the same symmetry it is sufficient to
   use a look-up table that covers only 1/4 of the period */
{int, int} static lookup_map256(int angle)
{
    int sign = 1;

    angle &= 0x3ff;

    if (angle > 768) {
        angle = 1024 - angle;
        sign = -1;
    } else if (angle > 512) {
        angle = angle - 512;
        sign = -1;
    } else if (angle > 256) {
        angle = 512 - angle;
    }
    /* nothing to do if angle <= 256 */

    return {sign, angle};
}

/* use LUT with 3rd harmonic included */
int sine_third_expanded(int angle)
{
    int sign;

    {sign, angle} = lookup_map256(angle);

    return sign * sine_third[angle];
}

/* use sine LUT */
int sine_table_expanded(int angle)
{
   int sign;

   {sign, angle} = lookup_map256(angle);

   return sign * sine_table[angle];
}

/**
 * \file arctan.xc
 * \brief Arc tangent calculations
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include "sine_table_big.h"
#include <stdlib.h>
#include <stdint.h>
#include <print.h>
#define COMMUTATION 1
#define SINE_TABLE  2

int arctg1(int Real, int Imag)
{
    int iAngleResult;
    int AbsX = abs(Real);
    int AbsY = abs(Imag);
    uint8_t cFlagOver45;
    int Mag;

    /* TODO: add comment */
#define defFactor 65536/64

    if(AbsY < AbsX) {
        cFlagOver45 = 0;
        AbsY *= defFactor;
        if (AbsX==0) {
            Mag = defFactor;
        } else {
            if (AbsY==0) {
                Mag = 0;
            } else {
                Mag = AbsY / AbsX;
            }
        }
    } else {
        cFlagOver45 = 1;
        AbsX *= defFactor;
        if (AbsY==0) {
            Mag = defFactor;
        } else {
            if (AbsX==0) {
                Mag = 0;
            } else {
                Mag = AbsX / AbsY;
            }
        }
    }

    if (Mag > 1023) {
        Mag = 1023;
    }

    iAngleResult = arctg_table[Mag];

    if (cFlagOver45) {
        iAngleResult = 1024 - iAngleResult;
    }

    if (Real >= 0) {   
        // 1. or 4. quadrant
        if (Imag  <  0) {
            iAngleResult = 4096 - iAngleResult;     // 4. quadrant
        }
        // if 1. quadrant everything is okay
    } else { 
        // if negativ 2. or 3. quadrant
        if (Imag < 0) {
            iAngleResult += 2048;                   // 3. quadrant
        } else {
            iAngleResult  = 2048 - iAngleResult;    // 2. quadrant
        }
    }
    return  iAngleResult;
}


/* FIXME: use a simple way to determine which part of the sine we want to look up */
int sine_expanded(int angle, int select_mode)
{
    int a1, a2;
    int sign = 0;
    a2 = angle >> 8;
    a1 = a2 >> 1;

    if(a1 == 1)
    {
        sign = -1;
        if(a2 == 3) {
            if(angle > 768) {
                angle = 1024 - angle;
                if(select_mode == COMMUTATION)
                    return sign * sine_third[angle];
                else if(select_mode == SINE_TABLE)
                    return sign * sine_table[angle];
            } else {
                angle = angle - 512;
                if(select_mode == COMMUTATION)
                    return sign * sine_third[angle];
                else if(select_mode == SINE_TABLE)
                    return sign * sine_table[angle];
            }
        }
        else if(a2 == 2)
        {
            angle = angle - 512;
            if(select_mode == COMMUTATION)
                return sign * sine_third[angle];
            else if(select_mode == SINE_TABLE)
                return sign * sine_table[angle];
        }
    }
    else if(a1 == 0)
    {
        sign = 0;
        if(a2 == 1) {
            if(angle > 256) {
                angle = 512 - angle;
                if(select_mode == COMMUTATION)
                    return sine_third[angle];
                else if(select_mode == SINE_TABLE)
                    return sine_table[angle];
            }
        }
    }
    if(sign < 0) {
        if(select_mode == COMMUTATION)
            return sign * sine_third[angle];
        else if(select_mode == SINE_TABLE)
            return sign * sine_table[angle];
    } else {
        sine_third[angle];
        if(select_mode == COMMUTATION)
            return sine_third[angle];
        else if(select_mode == SINE_TABLE)
            return sine_table[angle];
    }
}

/* use LUT with 3rd harmonics included */
int sine_third_expanded(int angle)
{
    return sine_expanded(angle, COMMUTATION);
}

/* use sine LUT */
int sine_table_expanded(int angle)
{
    return sine_expanded(angle, SINE_TABLE);
}

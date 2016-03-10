#include <foc_utilities.h>
#include <sine_table_foc.h>



int     check_limits( int input, int max_limit)
{
  if(input >=  max_limit) return (max_limit);
  if(input <= -max_limit) return (-max_limit);
  return(input);
}


int     calc_hysteresis_and_limit(int input, int hys_value, int max_value)
{
    if(input > 0)
    {
        if(input < hys_value) return(0);
        if(input > max_value) return(max_value);
        return(input);
    }
    if(input < 0)
    {
        if(input > -hys_value) return(0);
        if(input < -max_value) return(-max_value);
        return(input);
    }
 return(0);
}

// return value from 0 to 100 %
//
int check_tolerance(int actual_value, int compare_value)
{
int error      = 9999;
int quotient   = 0;

   if(compare_value == 0  || actual_value  == 0) return (error);

   if(compare_value > 0)
   {
   if(actual_value < 0) return (error);   // different sign = error
   quotient = (actual_value * 100) / compare_value;
   if(quotient >= 100) return(quotient - 100);
   if(quotient < 100)  return(100 - quotient);
   }
   else
   {
   if(actual_value > 0) return (error);  // different sign = error
   compare_value = -compare_value;
   actual_value  = -actual_value;
   quotient = (actual_value * 100) / compare_value;
   if(quotient >= 100) return(quotient - 100);
   if(quotient < 100)  return(100 - quotient);
   }
return(error);
}



int  calc_mean_one_periode (int iCX[], int iXX[], int new_value, int old_value, int speed_clock , int index_array)
{
int iRet=0;
int iIndex2;
int xx,ii;

iIndex2 = index_array + 16;

    xx = index_array;
    while(xx < iIndex2)
    {
        iCX[xx]++;                   // increment counter
        iXX[xx]+=new_value;          // add value
        xx++;
    }

   // if ((speed_clock & 0xC0) == 0) return(old_value); // 0x80 41 42 43 44 45, is not 0 if values are higher than 3F
    if ((speed_clock < 1) || (speed_clock > 6)) return(old_value); // reject invalid values


    //==========  new tick ==========================
  //  ii = speed_clock & 0x0F; //resulted in array indexes 0 to 5
    ii = (speed_clock & 0x7) - 1;//to have array indexes 0 to 5

        ii += index_array;
        if(iCX[ii])                      // to  avoid zero division
        { iRet = iXX[ii]/iCX[ii];}    // calc mean value

        iCX[ii]=0;  // clear counter
        iXX[ii]=0;  // clear sum register
        return(iRet);  // return new mean value
}




int low_pass_pt1_filter(int filter_sum[], int iFilterIndex, int iFactor,  int pt1_new_value)
{
int iRet;
        if(iFactor <= 1) return(pt1_new_value);

        filter_sum[iFilterIndex]     -= (filter_sum[iFilterIndex] / iFactor);
        filter_sum[iFilterIndex]     += pt1_new_value;
        iRet           = filter_sum[iFilterIndex]/iFactor;

return(iRet);
}




{int, int } cartesian_to_polar_conversion(int xValue, int yValue )
{
int iAbsX       =0;
int iAbsY       =0;
int iRatio      =0;
int iAmplitude  =0;
int iAngle      =0;

  if (xValue < 0 )
     {
    iAbsX = -xValue;
     }
     else
     {
          iAbsX = xValue;
     }

  if (yValue < 0 )
     {
    iAbsY = -yValue;
     }
     else
     {
       iAbsY = yValue;
     }

  if (iAbsY <= iAbsX)
     {
       if (iAbsX != 0)
       {
         iRatio = (iAbsY * 1024) / iAbsX;
       }

    if (iRatio > 1023)
       {
      iRatio=1023;
       }

    iAmplitude = (iAbsX * (int)vectortab[iRatio]) / 32768;   // >> 15;

       if ((xValue >= 0) && (yValue >= 0))              // Q1a X > Y  0 to 45
       {
         iAngle = (int)arctg_table[iRatio];
       }
            else if ((xValue < 0) && (yValue < 0))           // Q3a  180 to 225
            {
                 iAngle = 2048 + (int)arctg_table[iRatio];
            }
             else if ((xValue >= 0) && (yValue < 0))           // Q4b
              {
                   iAngle = 4096 - (int)arctg_table[iRatio];
              }
               else if ((xValue < 0) && (yValue >= 0))                 // Q2b
                {
                 iAngle = 2048 - (int)arctg_table[iRatio];
                }
            else
            {
              iAngle = 0;
            }
  }


  if(iAbsY > iAbsX)//  Y > X
     {
       if(iAbsY != 0)
          {
            iRatio = (iAbsX*1024) / iAbsY;
          }

       if(iRatio > 1023)
             {
               iRatio = 1023;
             }

       iAmplitude   = (iAbsY * (int)vectortab[iRatio])  / 32768;   // >> 15;

          if ((xValue >= 0) && (yValue > 0))
          {
               iAngle = 1024 - (int)arctg_table[iRatio];           // 90 to 46
          }
            else if ((xValue < 0) && (yValue < 0))
            {
                 iAngle = 3072 - (int)arctg_table[iRatio];
            }
             else if ((xValue >= 0) && (yValue < 0))
              {
                   iAngle = 3072 + (int)arctg_table[iRatio];
              }
                else if ((xValue < 0) && (yValue >= 0))
                {
                     iAngle = 1024 + (int)arctg_table[iRatio];
                }
            else
            {
              iAngle = 0;
            }
   }



  return {iAmplitude, iAngle};
}





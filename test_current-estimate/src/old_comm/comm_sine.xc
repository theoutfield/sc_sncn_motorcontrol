
#include <xs1.h>
#include <stdint.h>
#include <print.h>
#include <xscope.h>
#include "pwm_cli_inv.h"
#include "sine_table_big.h"
#include "refclk.h"
#include "predriver/a4935.h"
#include "dc_motor_config.h"
#include "hall_client.h"

//extern void cur_function(int a ,int b, int c);
//#define DC900

static t_pwm_control pwm_ctrl;

unsigned root_function(unsigned uSquareValue);
int max_31 =  2147483647;
void comm_sine_init_test(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = { 0, 0, 0 };  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



#define defRampMax 8192*2			//ramp params
#define defRampMin 32


void comm_sine_test( chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend signal_adc)
{
	unsigned cmd;
	unsigned pwm[3] = { 0, 0, 0 };
	int iIndexPWM, iPosFromHallOld=0;
	int iAngleFromHall  = 0;
	int iAngleUser      = 300;
	int iAngleFromRpm   = 0;
	int iAnglePWM, iActualSpeed = 0;
	int iPositionAbsolut=0;	int i;

	//int a1,a2,adc_a1,adc_a2,adc_a3, adc_a4,adc_b1,adc_b2,adc_b3,adc_b4;

	int umot =0, umot1 = 0, dir = 0, speed = 0, stop = 0, set = 0,limit=20;

	int iPwmOnOff = 1;
	int tt , tp;
	int ts, ts1;
	unsigned t2,t1, time;
 int dummy;

	//t:>time;
	while (1)
	{

		//{a1,a2,adc_a1,adc_a2,adc_a3, adc_a4,adc_b1,adc_b2,adc_b3,adc_b4}  = get_adc_calibrated_ad7949(c_adc, 1);
	 	// xscope_probe_data(0,a1);
	 	// xscope_probe_data(1,a2);

		//============= rotor position ===============================
		if(stop!=1)
		{
			//iPositionAbsolut = get_hall_absolute_pos(c_hall);
			// dummy= get_hall_speed(c_hall);
			// if(dummy)
			// iActualSpeed = (60000000/dummy)/POLE_PAIRS ;

			 iActualSpeed = get_speed_cal(c_hall);
			iAngleFromHall = get_hall_angle(c_hall);
			//{iActualSpeed, iAngleFromHall, iPositionAbsolut, dummy  } = get_hall_values(c_hall);
		}
		else
		{

			 iActualSpeed = get_speed_cal(c_hall);
			//{iActualSpeed, iAngleFromHall, iPositionAbsolut, dummy  } = get_hall_values(c_hall);
			iAngleFromHall=30;
			umot1=2000;
			if(set>limit)
			{
				set=limit;
			}
			//for(i=0;i<set;i++)
			{
			umot1=set*100;
			//t when timerafter (time + 1700500) :> time;
			}
		}


		iAngleFromRpm = iActualSpeed;
		if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;  // absolut value
		iAngleFromRpm *= 150;
		iAngleFromRpm /= 4000;


		if(umot1<0)
			dir = -1;
		if(umot1 >= 0)
			dir = 1;


		if (dir == 1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser + iAngleFromRpm  + 180;//100 M3  //100 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*umot1)/13889  + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;

		}

		if (dir == -1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser - iAngleFromRpm + 2700;  //2700 M3  //  2550 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;

		}



		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;


		if(iPwmOnOff==0)
		{ pwm[0]=0;   pwm[1]=0;   pwm[2]=0;  }

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);


		select {
			case c_commutation :> cmd:
			  if (cmd == 1) {
				 // speed = get_hall_speed(c_hall);
				 // c_commutation <: speed;
			  }
			  else if(cmd==3)
			  {
				  stop=1;
				  c_commutation :> set;
			  }
			  else if(cmd==2){ // set Umot
				  stop=0;
				  c_commutation :> umot1;   //val;
			  }
			  else if(cmd == 4) // set Direction
			  {
				  stop=0;
				  c_commutation :> dir;
			  }
			  break;
			default:
			  break;
		}// end select

	}

}


int get_sync(chanend c_sync)
{
	int pos;
	c_sync <: 20;
	c_sync :> pos;
	return pos;
}
void comm_sine_new( chanend c_commutation, chanend c_sync, chanend c_pwm_ctrl, chanend c_hall)
{
	unsigned cmd;
	unsigned pwm[3] = { 0, 0, 0 };
	int iIndexPWM, iPosFromHallOld=0;
	int iAngleFromHall  = 0;
	int iAngleUser      = 300;
	int iAngleFromRpm   = 0;
	int iAnglePWM, iActualSpeed;
	int iPositionAbsolut=0;	int i;

	int a1,a2,adc_a1,adc_a2,adc_a3, adc_a4,adc_b1,adc_b2,adc_b3,adc_b4;

	int umot =0, umot1 = 0, dir = 0, speed = 0, stop = 0, set = 0,limit=20;

	int iPwmOnOff = 1;
	int tt , tp;
	timer t; int ts, ts1;
	unsigned t2,t1, time;
 int dummy;

	t:>time;
	while (1)
	{

		//{a1,a2,adc_a1,adc_a2,adc_a3, adc_a4,adc_b1,adc_b2,adc_b3,adc_b4}  = get_adc_calibrated_ad7949(c_adc, 1);

		//============= rotor position ===============================
		if(stop!=1)
		{
			//iActualSpeed = get_hall_speed(c_hall);    // hall based
			//iAngleFromHall = get_hall_angle(c_sync);  // hall based

			iAngleFromHall = get_sync(c_sync);			 //sync based
			iAngleFromHall = (iAngleFromHall<<12)/500;


		}
		else
		{

			//iActualSpeed = get_hall_speed(c_hall);
			iAngleFromHall=30;
			umot1=2000;
			if(set>limit)
			{
				set=limit;
			}
			//for(i=0;i<set;i++)
			{
				umot1=set*100;
			//t when timerafter (time + 1700500) :> time;
			}
		}


		iAngleFromRpm = iActualSpeed;
		if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;  // absolut value
		iAngleFromRpm *= 150;
		iAngleFromRpm /= 4000;


		if(umot1<0)
			dir = -1;
		if(umot1 >= 0)
			dir = 1;


		if (dir == 1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser + iAngleFromRpm  + 180;//100 M3  //100 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*umot1)/13889  + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;

		}

		if (dir == -1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser - iAngleFromRpm + 2700;  //2700 M3  //  2550 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;

		}



		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;


		if(iPwmOnOff==0)
		{ pwm[0]=0;   pwm[1]=0;   pwm[2]=0;  }

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);


		select {
			case c_commutation :> cmd:
			  if (cmd == 1) {
				 // speed = get_hall_speed(c_hall);
				 // c_commutation <: speed;
			  }
			  else if(cmd==3)
			  {
				  stop=1;
				  c_commutation :> set;
			  }
			  else if(cmd==2){ // set Umot
				  stop=0;
				  c_commutation :> umot1;   //val;
			  }
			  else if(cmd == 4) // set Direction
			  {
				  stop=0;
				  c_commutation :> dir;
			  }
			  break;
			default:
			  break;
		}// end select

	}

}






void commutation_test(chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend signal_adc)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init_test(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;




	 /* signal_adc <: 1;
	  while(1)
	  {
		  unsigned cmd, found =0;
		  select
		  {
			case signal_adc :> cmd:
				found = 1;
				break;
			default:
				break;
		  }
		  if(found == 1)
			  break;
	  }
*/
	 // do_adc_calibration_ad7949(c_adc);
	  comm_sine_test( c_commutation, c_hall, c_pwm_ctrl, signal_adc);
	  //comm_sine_new( c_commutation, c_hall, c_pwm_ctrl, c_adc);
}


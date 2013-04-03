
#include<comm_loop.h>

unsigned pwm[3] = { 0, 0, 0 };

void space_vector_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff )
{
		// pwm 13889 * 4 nsec = 55.556 usec  18Khz

		iIndexPWM = iIndexPWM >> 2;

		pwm[0] = ((SPACE_TABLE[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM +341) & 0x3ff;
		pwm[1] = ((SPACE_TABLE[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM + 342) & 0x3ff;
		pwm[2] = ((SPACE_TABLE[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;


		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;


		if(iMotHoldingTorque == 0)
		if(iPwmOnOff==0)
		{ pwm[0]= 0;   pwm[1]=0;   pwm[2]=0;  }

		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

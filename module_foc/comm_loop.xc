/**
 * Module:  module_comm_loop
 * Ludwig Orgler orgler@tin.it synapticon 04/2013
  *
 **/
#include <xs1.h>
#include <stdint.h>
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "comm_loop.h"

unsigned pwm[3] = { 0, 0, 0 };
static t_pwm_control pwm_ctrl;
static t_foc_control focx;



void comm_sine(chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_encoder, chanend c_pwm_ctrl )
{
int ii;


		while (1)
		{

//======================== read current ========================================================================
		#ifdef DC900
		{focx.a1, focx.a2,focx.a1_calib,focx.a2_calib,focx.adcTemperature1,focx.adcTemperature2,focx.adcVoltageSupply,focx.adcDummy,focx.adcExternPoti1,focx.adcExternPoti2}=
				get_adc_calibrated_ad7949(c_adc, focx.iPwmOnOff); //get_adc_vals_raw_ad7949(adc);
		#endif



			#ifdef DC100
			 {a1, a2, a3}  = get_adc_vals_calibrated_int16_ltc1408( c_adc );
			#endif
//======================== end current =========================================================================


//============================== rotor position  from hall or encoder ==========================================
#ifdef defHALL
		{focx.iHallSpeed,focx.iHallAngle,focx.iHallPosition,focx.iHallPinState}  = get_hall_values(c_hall);
#endif

#ifdef defENCODER
		{focx.iEncoderSpeed,focx.iEncoderAngle,focx.iEncoderPosition,focx.iEncoderPinState} = get_encoder_values(c_encoder);
#endif


        function_FOC_CONTROL(focx, c_commutation);



				  	 if(focx.iPwmOnOff==0)
				  	 { pwm[0]= 0;   pwm[1]=0;   pwm[2]=0;  }
				  	 else
				  	 {
				  	  ii = (focx.iAnglePWM & 0x0FFF) >> 2;

					  // pwm 13889 * 4 nsec = 55,556µsec  18Khz
					  pwm[0] = ((SPACE_TABLE[ii])* focx.iUmotMotor)/4096   + PWM_MAX_VALUE/2;
					  ii = (ii +341) & 0x3ff;
					  pwm[1] = ((SPACE_TABLE[ii])* focx.iUmotMotor)/4096   + PWM_MAX_VALUE/2;
					  ii = (ii + 342) & 0x3ff;
					  pwm[2] = ((SPACE_TABLE[ii])* focx.iUmotMotor)/4096   + PWM_MAX_VALUE/2;

					  if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
					  if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
					  if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;
				    }

					update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

	}// end while(1)
}// end function comm_sine


void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

void commutation(chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_encoder, chanend c_pwm_ctrl )
{
	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  t :> ts;
	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  comm_sine(c_adc, c_commutation, c_hall, c_encoder, c_pwm_ctrl );
}


void    function_FOC_CONTROL(t_foc_control& focx, chanend c_commutation)
{

}









#include <xs1.h>
#include <stdint.h>
#include "hall_client.h"
#include "refclk.h"
#include "hall_config.h"

int get_hall_position(chanend c_hall)
{
  int32_t pos;
  c_hall <: HALL_POS_REQ;
  c_hall :> pos;  				// 6 steps angle

  return pos;
}

int get_hall_position_absolute(chanend c_hall)
{
  int pos;
  c_hall <: HALL_ABSOLUTE_POS_REQ;
  c_hall :> pos; 				// position with dirn

  return pos;
}

int get_hall_speed_cal(chanend c_hall)
{
  int32_t speed;
  c_hall <: HALL_VELOCITY_REQ;
  c_hall :> speed;  			// speed

  return speed;
}

int get_hall_velocity(chanend c_hall, hall_par &hall_params)
{
	int speed = 0, time;
	time = get_hall_speed_cal(c_hall);
	if(time)
		speed = (RPM_CONST/time)/ hall_params.pole_pairs;

	return speed;

}

int _get_hall_velocity_pwm_resolution(chanend c_hall, hall_par &hall_params)
{
	int velocity;
	//hall_params.pole_pairs*4095*(1)
	c_hall <: HALL_VELOCITY_PWM_RESOLUTION_REQ;
	c_hall :> velocity;
											//ms constant
	velocity = ((velocity/FILTER_LENGTH_HALL) * 18000 * 60)/(hall_params.pole_pairs * 4095 *1);
	return velocity;
}

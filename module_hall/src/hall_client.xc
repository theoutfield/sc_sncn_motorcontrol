#include "hall_client.h"

int get_hall_position(chanend c_hall)
{
  int position;
  c_hall <: HALL_POS_REQ;
  c_hall :> position;
  return position;
}

{int , int} get_hall_position_absolute(chanend c_hall)
{
  int position;
  int direction;
  c_hall <: HALL_ABSOLUTE_POS_REQ;
  c_hall :> position;
  c_hall :> direction;
  return {position, direction};
}

int get_hall_velocity(chanend c_hall, hall_par &hall_params)
{
	int velocity;
	c_hall <: HALL_VELOCITY_REQ;
	c_hall :> velocity;
	velocity = ((velocity/FILTER_LENGTH_HALL) * 1000 * 60)/(hall_params.pole_pairs * 4095 *1);
	return velocity;
}

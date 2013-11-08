
#include <stdio.h>
#include <math.h>

#ifndef _PROFILE_H_
#define _PROFILE_H_
/*Profile Velocity Quick Stop*/

/**
 * \brief Initialise Quick Stop Velocity Profile
 *
 *  Input
 * \param actual_velocity
 * \param quick_stop_deceleration defines the deceleration for quick stop profile
 *
 * Output
 * \return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);

/**
 * \brief Generate Quick Stop Velocity Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target velocity at the step input
 */
extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/

/**
 * \brief Initialise Velocity Profile
 *
 *  Input
 * \param target_velocity
 * \param actual_velocity
 * \param acceleration for the velocity profile
 * \param deceleration for the velocity profile
 * \param max_velocity for the velocity profile
 *
 * Output
 * \return no. of steps for velocity profile : range [1 - steps]
 */
extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration, int max_velocity);

/**
 * \brief Generate Velocity Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target velocity at the step input
 */
extern int velocity_profile_generate(int step);

/*Profile Position Mode*/

/**
 * \brief Initialise Position Profile Limits
 *
 *  Input
 * \param gear_ratio
 * \param max_acceleration for the position profile
 * \param max_velocity for the position profile
 *
 */
extern void init_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity);


/**
 * \brief Initialise Position Profile
 *
 *  Input
 * \param target_position
 * \param actual_position
 * \param velocity for the position profile
 * \param acceleration for the position profile
 * \param deceleration for the position profile
 *
 * Output
 * \return no. of steps for position profile : range [1 - steps]
 */
extern int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
        						 int deceleration);


/**
 * \brief Generate Position Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target position at the step input
 */
extern int position_profile_generate(int step);

/*Profile Position Quick Stop*/

/**
 * \brief Initialise Quick Stop Position Profile
 *
 *  Input
 * \param actual_velocity
 * \param actual_position
 * \param max_acceleration defines the deceleration for quick stop profile
 *
 * Output
 * \return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration);

/**
 * \brief Generate Quick Stop Position Profile
 *
 *  Input
 * \param step current step of the profile
 * \param actual_velocity
 *
 * Output
 * \return corresponding target position at the step input
 */
extern int quick_stop_position_profile_generate(int steps, int actual_velocity);


extern int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration, int max_value);


extern int  linear_profile_generate(int step);



#endif /* _PROFILE_H_ */

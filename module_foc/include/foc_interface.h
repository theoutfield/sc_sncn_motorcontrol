#pragma once

//------------- ff ==>> filter enum index used with  filter_sum[8] --------------------------
enum
{
fffield,
ff_field_out,
ffCurrVector,
ffSpeedActual,
ffAngleRPM,
ffhalltime,
fftorque,
};

/*===========================================
 *
 * input: field_actual
 *
 * D-value for invers-park
 */
int field_control(int field_new, int field_e1, int field_e2, int q_value, int field_out_p_part, int field_out_i_part, int par_field_kp, int par_field_ki, int field_out1, int field_out2, int filter_sum[]);



// ===  (a,b,c)(a,b) (the Clarke transformation) which outputs a two co-ordinate time variant system
//
{int,int}  clarke_transformation(int curr_phase_b, int curr_phase_c);



// === (a,b) (d,q) (the Park transformation) which outputs a two co-ordinate time invariant system
//
{int, int} park_transformation(int sinus_tab, int cosinus_tab, int alpha, int beta );



// The (d,q)->(a,b) projection (inverse Park transformation)
// The outputs of this block are the components of the reference vector
// This is the voltage space vector to be applied to the motor phases.
//
{int, int} invers_park(int sinus_tab, int cosinus_tab, int field, int torque);



/*===== adjust pwm-angle with RPM like a phase shifting ======
 * as higher the output frequency as higher the inductive reactance
 * to
 *
 */
int    adjust_angle_reference_pwm(int angle_inv_park, int angle_offset, int measure_tick, int speed_actual, int q_value, int filter_sum[]);










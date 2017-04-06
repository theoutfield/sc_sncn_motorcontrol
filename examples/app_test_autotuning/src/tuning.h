/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <platform.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <refclk.h>
#include <adc_service.h>
#include <position_ctrl_service.h>

#include <xscope.h>
#include <mc_internal_constants.h>
#include <position_feedback_service.h>
#include <data_processing.h>
#include <random_config.h>
#include <random.h>


#define CONTROL_DISABLE 0
#define POSITION_CONTROL_ENABLE 1
#define VELOCITY_CONTROL_ENABLE 2


typedef enum {NON_COMPETITOR, COMPETITOR} gender ;
typedef enum {LOSER, WINNER, NEUTRAL} ;

#define RANGE_KP 20000000
#define RANGE_KI 1000000

#define NUMBER_OF_TESTS MAX_ROWS
#define NUMBER_OF_GENERATIONS 5
#define GENERATION_SIZE 100
#define NUMBER_OF_WINNERS 5
#define NUMBER_OF_COMPETITORS 20
#define NUMBER_OF_LOSERS (NUMBER_OF_COMPETITORS-NUMBER_OF_WINNERS)


struct individual
{
    unsigned int kp;
    unsigned int ki;
    int criterion;
    unsigned int energy;
    gender competitor;
    int status;
    int age;
    int overshoot;
    int oscillation;
};


interface PositionLimiterInterface {
    void set_limit(int limit);
    int get_limit();
};

interface TuningStepInterface {


  void set_reference_velocity(int in_velocity);
  int get_reference_velocity();

  void set_reference_velocity_display(int in_velocity);
  int get_reference_velocity_display();

  void set_reference_position(int in_position);
  int get_reference_position();

  void set_reference_position_display(int in_position);
  int get_reference_position_display();


  void set_time_zero(unsigned int in_time_zero);
  unsigned int get_time_zero();

  void set_time_reference(unsigned int in_time_reference);
  unsigned int get_time_reference();

  void set_ctrl_parameters(PosVelocityControlConfig in_pos_velocity_ctrl_config);
  PosVelocityControlConfig get_ctrl_parameters();

  void start_steps(int in_flag);
  void stop_steps();
  unsigned int check_flag();

};

void tuning_step_service ( interface TuningStepInterface server i_tuning_step[3]);
void user_interface(client interface PositionVelocityCtrlInterface i_position_control, client interface TuningStepInterface i_tuning_step);
void make_steps(client interface PositionVelocityCtrlInterface i_position_control, client interface TuningStepInterface i_tuning_step, client interface PositionFeedbackInterface i_position_feedback_interface);
struct individual autotune (client interface PositionVelocityCtrlInterface i_position_control, int number_of_generations, client interface TuningStepInterface i_tuning_step, client interface PositionFeedbackInterface i_position_feedback);

void compute_pid(struct individual * generation, unsigned int index, client interface PositionVelocityCtrlInterface i_position_control, client interface TuningStepInterface i_tuning_step);
void test_crossover();


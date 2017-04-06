/*
 * tuning.xc
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */
#include <tuning.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <user_config.h>
#include <math.h>

void init_generation (struct individual * generation, random_generator_t gen)
{
    int Kp_max = RANGE_KP;
    int Ki_max = RANGE_KI;
    int Kp = VELOCITY_Kp;
    int Ki = VELOCITY_Ki;

    int number_kp = sqrt(NUMBER_OF_COMPETITORS)+1;

    for (int i =0; i<GENERATION_SIZE; i++)
    {
        if (i < NUMBER_OF_COMPETITORS)
        {
//            generation[i].kp = (1+i/4)*RANGE_KP/(4);
//            generation[i].ki = (1+(i%(4)))*RANGE_KI/(4);
            generation[i].kp = random_get_random_number(gen)%RANGE_KP;
            generation[i].ki = random_get_random_number(gen)%RANGE_KI;
//            generation[i].kp = 3500000;
//            generation[i].ki = 45000;
            generation[i].competitor = COMPETITOR;
//            printf ("[%d] Kp : %d ; Ki : %d \n", i,generation[i].kp,generation[i].ki);
        }
        else {
            generation[i].kp = random_get_random_number(gen)%RANGE_KP;
            generation[i].ki = random_get_random_number(gen)%RANGE_KI;
            generation[i].competitor = NON_COMPETITOR;
        }


        generation[i].status = NEUTRAL;
        generation[i].age = 0;
//        gen = random_create_generator_from_hw_seed();
    }
}

int auto_offset(interface MotorcontrolInterface client i_motorcontrol)
{
    printf("Sending offset_detection command ...\n");
    i_motorcontrol.set_offset_detection_enabled();

    while(i_motorcontrol.set_calib(0)==-1) delay_milliseconds(50);//wait until offset is detected

    int offset=i_motorcontrol.set_calib(0);
    printf("Detected offset is: %i\n", offset);
    //    printf(">>  CHECK PROPER OFFSET POLARITY ...\n");
    int proper_sensor_polarity=i_motorcontrol.get_sensor_polarity_state();
    if(proper_sensor_polarity == 1) {
        printf(">>  PROPER POSITION SENSOR POLARITY ...\n");
    } else {
        printf(">>  WRONG POSITION SENSOR POLARITY ...\n");
    }
    return offset;
}

void evaluate_fitness (struct individual * generation)
{

    int energy_min [NUMBER_OF_WINNERS];
    int index [NUMBER_OF_WINNERS] = {0};

    for (int i = 0; i < NUMBER_OF_WINNERS; i++)
    {
        energy_min[i]=INT_MAX;
    }

    // Evaluates the winners and losers, the losers will be replaced in the next generation
    for(int i=0; i < GENERATION_SIZE; i++)
    {
        if(generation[i].competitor == COMPETITOR)
        {
            for (int j = 0; j < NUMBER_OF_WINNERS; j++)
            {
                if (generation[i].criterion < energy_min[j])
                {

                    (generation[index[NUMBER_OF_WINNERS-1]]).status = LOSER;
                    generation[i].status = WINNER;
                    for (int index_winner = NUMBER_OF_WINNERS-2; index_winner >= j ; index_winner --)
                    {
                        index[index_winner+1] = index[index_winner];
                        energy_min[index_winner+1]= energy_min[index_winner];
//                        printf("max %d :  %d => %d\n ",index_winner, index[index_winner+1], index[index_winner] );
                    }
                    index[j]=i;
                    energy_min[j]= generation[i].criterion;
                    break;
                }
                else  generation[i].status = LOSER;
            }
        }
    }
    for (int i = 0; i < NUMBER_OF_WINNERS; i++)
    {
        printf("%d°: %d ; Kp => %d ; Ki => %d ; energy => %d ; age => %d\n", i+1, index[i],generation[index[i]].kp, generation[index[i]].ki,  energy_min[i], generation[index[i]].age);
    }
}


void generate_children(struct individual * generation, int index_partner_1, int index_partner_2, random_generator_t gen)
{

    int index_child = -1;
    // Find two children in the list for each couple
    int weight_percent_kp = random_get_random_number(gen)%100; // ratio of genes from each parents into the child
    if (weight_percent_kp < 20) weight_percent_kp += 25;
    if (weight_percent_kp > 80) weight_percent_kp -= 25;

    int weight_percent_ki = random_get_random_number(gen)%100;
    if (weight_percent_ki < 20) weight_percent_ki += 25;
    if (weight_percent_ki > 80) weight_percent_ki -= 25;
//    struct individual generation [GENERATION_SIZE];
    for (int number_of_children = 0; number_of_children < 2; number_of_children++)
    {
        do{
            index_child++;
        }while(generation[index_child].status && index_child < (GENERATION_SIZE-2));
        if(!generation[index_child].status)
        {
            // Make a weighted average of the parameters of each parents to compute the ones of the child

            if (!number_of_children)
            {
                generation[index_child].kp = (weight_percent_kp*generation[index_partner_1].kp + (100-weight_percent_kp)*generation[index_partner_2].kp)/100;
                generation[index_child].ki = (weight_percent_ki*generation[index_partner_1].ki + (100-weight_percent_ki)*generation[index_partner_2].ki)/100;
            }else
            {
                generation[index_child].kp = (weight_percent_kp*generation[index_partner_2].kp + (100-weight_percent_kp)*generation[index_partner_1].kp)/100;
                generation[index_child].ki = (weight_percent_ki*generation[index_partner_2].ki + (100-weight_percent_ki)*generation[index_partner_1].ki)/100;
            }
//            printf("Child n°%d of %d and %d with random 0.%d and 0.%d : Kp = %d, Ki = %d\n",index_child, index_partner_1, index_partner_2,weight_percent_kp,weight_percent_ki,  generation[index_child].kp , generation[index_child].ki);
            generation[index_child].status = NEUTRAL;
            generation[index_child].competitor = COMPETITOR;
            generation[index_child].age = 0;
        }
    }
}


void crossover(struct individual *generation, random_generator_t gen)
{
//    random_generator_t gen = random_create_generator_from_hw_seed();
    int child_nb=0;
    evaluate_fitness(generation);


    // Crossover
    int index_partner_1, index_partner_2;
    for (index_partner_1 = 0; index_partner_1 < GENERATION_SIZE; index_partner_1++)
    {
        // Get the first partner from the list of winners
        if (generation[index_partner_1].status == WINNER)
        {
            generation[index_partner_1].status = NEUTRAL;
            //Get the second partner randomly
            do {
                index_partner_2 = random_get_random_number(gen)%GENERATION_SIZE;
            }while (!generation[index_partner_2].status || index_partner_2 == index_partner_1);
            child_nb++;
//            printf("Child number %d :\n", child_nb);
            generate_children(generation, index_partner_1, index_partner_2, gen);
        }
    }

    // Check if there are any room left in next generation => Losers to replace
    for (int index_loser = 0; index_loser < GENERATION_SIZE; index_loser++)
    {

        if (generation[index_loser].status == LOSER)
        {
            // Get both partners randomly
            do {
                index_partner_1 = rand()%GENERATION_SIZE;
            }while (!generation[index_partner_1].status);
            do {
                index_partner_2 = rand()%GENERATION_SIZE;
            }while (!generation[index_partner_2].status||index_partner_2 == index_partner_1);

            child_nb++;
//            printf("Child number %d :\n", child_nb);
            generate_children(generation, index_partner_1, index_partner_2, gen);
        }
    }
}

void test_crossover()
{

    random_generator_t gen = random_create_generator_from_hw_seed();
    struct individual generation [GENERATION_SIZE];
    printf("First Generation : \n");
    int energy [GENERATION_SIZE] = {6176, 4850, 4198, 3672, 8642, 4942, 5034, 2443, 7822, 3308, 1739, 4848, 7113, 4166, 7454, 2,2,2,2,512};
    for (int i =0; i < GENERATION_SIZE; i++)
    {
        generation[i].ki=100 * i;
        generation[i].kp = 100 * (GENERATION_SIZE - i);
        if (i < NUMBER_OF_COMPETITORS)
        {
            generation[i].kp = (1+i/4)*RANGE_KP/4;
            generation[i].ki = (1+(i%4))*RANGE_KI/4;
            generation[i].competitor = COMPETITOR;
        }
        else
        {
            generation[i].kp = random_get_random_number(gen)%RANGE_KP;
            generation[i].ki = random_get_random_number(gen)%RANGE_KI;
            generation[i].competitor = NON_COMPETITOR;
        }
        generation[i].criterion = energy[i];
//        printf("%d : KI => %d, KP => %d, energy => %d\n", i, generation[i].ki, generation[i].kp, generation[i].criterion);
        generation[i].status = NEUTRAL;

    }

    crossover (generation, gen);
    for (int i =0; i < GENERATION_SIZE; i++)
    {
        printf("%d : KI => %d, KP => %d, energy => %d\n", i, generation[i].ki, generation[i].kp, generation[i].criterion);
    }
}

void tuning_step_service(interface TuningStepInterface server i_tuning_step[3])

{
    int reference_velocity = 1000;
    int reference_position = 1000;
    int velocity_display = reference_velocity;
    int position_display = reference_position;
    unsigned int time_ms_zero = 1000;
    unsigned int time_ms_reference = 1000;
    unsigned int flag = 0;
    PosVelocityControlConfig pos_velocity_ctrl_config;
    while(1)
    {
        select{
        case i_tuning_step[int i].set_reference_velocity(int in_velocity):
                reference_velocity = in_velocity;
            break;

        case i_tuning_step[int i].get_reference_velocity() -> int out_velocity :
                out_velocity = reference_velocity;
            break;
        case i_tuning_step[int i].set_reference_velocity_display(int in_velocity):
                velocity_display = in_velocity;
            break;

        case i_tuning_step[int i].get_reference_velocity_display() -> int out_velocity :
                out_velocity = velocity_display;
            break;

        case i_tuning_step[int i].set_reference_position(int in_position):
                reference_position = in_position;
            break;

        case i_tuning_step[int i].get_reference_position() -> int out_position :
                out_position = reference_position;
            break;
        case i_tuning_step[int i].set_reference_position_display(int in_position):
                position_display = in_position;
            break;

        case i_tuning_step[int i].get_reference_position_display() -> int out_position :
                out_position = position_display;
            break;

        case i_tuning_step[int i].set_time_zero(unsigned int in_time_zero):
                time_ms_zero = in_time_zero;
            break;

        case i_tuning_step[int i].get_time_zero() -> unsigned int out_time_zero :
                out_time_zero = time_ms_zero;
            break;

        case i_tuning_step[int i].set_time_reference(unsigned int in_time_reference):
                time_ms_reference = in_time_reference;
            break;

        case i_tuning_step[int i].get_time_reference() -> unsigned int out_time_reference:
                out_time_reference = time_ms_reference;
                break;

        case i_tuning_step[int i].set_ctrl_parameters(PosVelocityControlConfig in_pos_velocity_ctrl_config):
                pos_velocity_ctrl_config = in_pos_velocity_ctrl_config;
        break;

        case i_tuning_step[int i].get_ctrl_parameters() -> PosVelocityControlConfig out_pos_velocity_ctrl_config:
                out_pos_velocity_ctrl_config = pos_velocity_ctrl_config;
        break;

        case i_tuning_step[int i].start_steps(int in_flag) :
                flag = in_flag;
        break;

        case i_tuning_step[int i].stop_steps() :
                flag = 0;
        break;

        case i_tuning_step[int i].check_flag() -> unsigned int out_flag:
                out_flag = flag;
        break;
        }

    }
}

void make_steps(client interface PositionVelocityCtrlInterface i_position_control, client interface TuningStepInterface i_tuning_step,
        client interface PositionFeedbackInterface i_position_feedback)
{
    unsigned int time_ms_zero, time_ms_reference;
    int velocity = 0, position = 0;
    int flag = 0;
    int count;
    int status, pos;
    int temp_kp, temp_ki, temp_kd;
    DownstreamControlData downstream_control_data;
    PosVelocityControlConfig pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
    while(1)
    {
        time_ms_zero = i_tuning_step.get_time_zero();
        i_tuning_step.set_reference_velocity_display(0);
        i_tuning_step.set_reference_position_display(0);
        if (flag == VELOCITY_CONTROL_ENABLE)
        {
            pos_velocity_ctrl_config.P_velocity = VELOCITY_Kp ;
            pos_velocity_ctrl_config.I_velocity = VELOCITY_Ki ;
            pos_velocity_ctrl_config.D_velocity = VELOCITY_Kd ;
            i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
            i_position_control.enable_velocity_ctrl();
            downstream_control_data.velocity_cmd = 0;
            i_position_control.update_control_data(downstream_control_data);
            delay_milliseconds(time_ms_zero/4);
            pos_velocity_ctrl_config = i_tuning_step.get_ctrl_parameters();
            i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
        }
        i_position_control.enable_torque_ctrl();
        downstream_control_data.offset_torque = 0;
        downstream_control_data.torque_cmd = 0;

        i_position_control.update_control_data(downstream_control_data);


        flag = i_tuning_step.check_flag();
        if (flag)
        {

            if (flag == VELOCITY_CONTROL_ENABLE)
            {
                delay_milliseconds(3*time_ms_zero/4);
                i_position_control.enable_velocity_ctrl();
//                downstream_control_data.velocity_cmd = 0;
//                delay_milliseconds(200);
                velocity = i_tuning_step.get_reference_velocity();
                i_tuning_step.set_reference_velocity_display(velocity);
                i_position_control.enable_velocity_ctrl();
                downstream_control_data.velocity_cmd = velocity;
            }
            else if (flag == POSITION_CONTROL_ENABLE)
            {
                delay_milliseconds(time_ms_zero);
                i_position_feedback.set_position(0);
                {count, pos, status} = i_position_feedback.get_position();
                printf("Count : %d\n", count);
                position = i_tuning_step.get_reference_position();
                i_tuning_step.set_reference_position_display(position);
                i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);
                downstream_control_data.position_cmd = position;
            }

            i_position_control.update_control_data(downstream_control_data);
            time_ms_reference = i_tuning_step.get_time_reference();
            delay_milliseconds(time_ms_reference);

        }

    }
}

void compute_pid(struct individual * generation, unsigned int index, client interface PositionVelocityCtrlInterface i_position_control,
        client interface TuningStepInterface i_tuning_step)
{
    PosVelocityControlConfig pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
    pos_velocity_ctrl_config.P_velocity = generation[index].kp;
    pos_velocity_ctrl_config.I_velocity = generation[index].ki;
    pos_velocity_ctrl_config.D_velocity = 0;
    i_tuning_step.set_ctrl_parameters(pos_velocity_ctrl_config);
    pos_velocity_ctrl_config = i_tuning_step.get_ctrl_parameters();
}

struct individual autotune (client interface PositionVelocityCtrlInterface i_position_control, int number_of_generations,
        client interface TuningStepInterface i_tuning_step, client interface PositionFeedbackInterface i_position_feedback)
{
    int velocity;
    int velocity_command = 300;
    int error_position = 0, error_velocity = 0;
    int energy_error_position = 0, energy_error_velocity = 0;
    unsigned int time_reference = 400, time_zero = 300;
    int error_min = 0;
    int oscillation_number = 0, overshoot = 0;
    int sign = 0;
    int previous_velocity = 0, acceleration;
    int overshoot_average = 0;
    int oscillation_average = 0;

    random_generator_t gen = random_create_generator_from_hw_seed();

    struct individual generation [GENERATION_SIZE];

    init_generation(generation, gen);
    i_tuning_step.set_reference_velocity(velocity_command);
    i_tuning_step.set_time_zero(time_zero);
    i_tuning_step.set_time_reference(time_reference);
    i_tuning_step.start_steps(VELOCITY_CONTROL_ENABLE);
    for (int generation_number = 0; generation_number < number_of_generations; generation_number++)
    {
        for (int i =0; i<NUMBER_OF_COMPETITORS; i++)
        {
            energy_error_velocity = 0;
            error_min = 0;
            oscillation_number = 0;
            compute_pid(generation, i, i_position_control, i_tuning_step);
            while (!i_tuning_step.get_reference_velocity_display());

            while (i_tuning_step.get_reference_velocity_display())
            {
                velocity = i_position_feedback.get_velocity();
                acceleration = velocity - previous_velocity;
                previous_velocity = velocity;
                velocity_command = i_tuning_step.get_reference_velocity();
                error_velocity = velocity_command-velocity;
                energy_error_velocity += (error_velocity*error_velocity);
                if (error_velocity < error_min)
                    error_min = error_velocity;
                if ((acceleration > 0) != sign)
                {
                    oscillation_number++;
                    sign ^= 1;
                }
                delay_microseconds(1000);
            }
            int age = generation[i].age;
            overshoot = abs(1000 * error_min / velocity_command);
            if (age)
            {
                generation[i].overshoot = (generation[i].overshoot*age+overshoot)/(age+1);
                generation[i].oscillation = (generation[i].oscillation*age+oscillation_number)/(age+1);
                generation[i].energy = (generation[i].energy*age+energy_error_velocity)/(age+1);
            }
            else {
                generation[i].overshoot = overshoot;
                generation[i].oscillation = oscillation_number;
                generation[i].energy = energy_error_velocity;
            }

//            printf("energy_error_velocity : %d\n", energy_error_velocity);
        }

        for(int i = 0; i< GENERATION_SIZE; i++)
        {
            if (generation[i].competitor)
            {
                overshoot_average += generation[i].overshoot;
                oscillation_average += generation[i].oscillation;
            }
        }
        overshoot_average /= NUMBER_OF_COMPETITORS;
        oscillation_average /= NUMBER_OF_COMPETITORS;

        int total_criterion;

        for(int i = 0; i< GENERATION_SIZE; i++)
        {
            if (generation[i].competitor)
            {
            printf("[%d] :Energy : %d ; Overshoot : %d ; Vibration : %d\n", i, generation[i].energy, generation[i].overshoot, generation[i].oscillation);
                total_criterion = generation[i].energy*2.9 + (generation[i].overshoot) * /*200000*/0 + (generation[i].oscillation - oscillation_average) * /*200000*/0;
//                total_criterion += (generation_number-generation[i].age)*250000;
//                printf ("[%d] generation : %d, age : %d\n", i, generation_number, generation[i].age);
                generation[i].age++;
                generation[i].criterion = total_criterion;
            }

        }


        if (!generation_number)
        {
            printf("\nFirst Generation : \n");
        }
        else if(generation_number == 1)
            printf("\nSecond generation : \n");
        else printf("\nGeneration n°%d : \n", generation_number+1);
        crossover(generation, gen);

    }

    evaluate_fitness(generation);
    int index_best_fit = 0;
    int i = 0;
    while(1){
        while (!i_tuning_step.get_reference_velocity_display());

        if(generation[i].status == WINNER)
            compute_pid(generation, i, i_position_control, i_tuning_step);
        while (i_tuning_step.get_reference_velocity_display());

        i++;
        if (i == NUMBER_OF_COMPETITORS) i=0;
    }

    for (int i = 0 ; i < NUMBER_OF_COMPETITORS; i++ )
    {
        if(generation[i].status == WINNER)
        {
            if (generation[i].age > generation[index_best_fit].age)
            {
                index_best_fit = i;

            }else if(generation[i].age == generation[index_best_fit].age)
            {
                if(generation[i].criterion < generation[index_best_fit].criterion )
                    index_best_fit = i;
            }
        }

    }
    compute_pid(generation, index_best_fit, i_position_control, i_tuning_step);
    return generation[index_best_fit];
}



void user_interface(client interface PositionVelocityCtrlInterface i_position_control, client interface TuningStepInterface i_tuning_step)
{
    delay_milliseconds(500);
    printf(">>   SOMANET PID TUNING SERVICE STARTING...\n");

    DownstreamControlData downstream_control_data;
    PosVelocityControlConfig pos_velocity_ctrl_config;

    MotorcontrolConfig motorcontrol_config;

    fflush(stdout);
    //read and adjust the offset.
    while (1)
    {
        char mode = '@';
        char mode_2 = '@';
        char mode_3 = '@';
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n')
        {
            if(isdigit(c)>0)
            {
                value *= 10;
                value += c - '0';
            }
            else if (c == '-')
            {
                sign = -1;
            }
            else if (c != ' ')
            {
                if (mode == '@')
                {
                    mode = c;
                }
                else if (mode_2 == '@')
                {
                    mode_2 = c;
                }
                else
                {
                    mode_3 = c;
                }
            }
            delay_milliseconds(100);
        }
        value *= sign;

        switch(mode)
        {

        //Start/Stop commands
        case 's':
            switch(mode_2)
            {
            case 'p':
                i_tuning_step.start_steps(POSITION_CONTROL_ENABLE);
                break;
            case 'v' :
                i_tuning_step.start_steps(VELOCITY_CONTROL_ENABLE);
                break;
            default :
                if (value) i_tuning_step.start_steps(value);
                else i_tuning_step.stop_steps();
                break;
            }
            break;

        //position commands
        case 'p':
            i_tuning_step.set_reference_position(value);

            break;
            //velocity commands
        case 'v':
            i_tuning_step.set_reference_velocity(value);

            break;

        //enable and disable torque controller
        case 't':
            switch(mode_2)
            {
            case 'z':
                i_tuning_step.set_time_zero(value);
                break;


            case 'r':
                i_tuning_step.set_time_reference(value);
                break;
            }

        break;

        //reverse torque
        case 'r':
                downstream_control_data.torque_cmd = -downstream_control_data.torque_cmd;
//                if(velocity_running)
//                {
//                    velocity = -velocity;
//                    downstream_control_data.offset_torque = 0;
//                    downstream_control_data.velocity_cmd = velocity;
//                    i_position_control.update_control_data(downstream_control_data);
//                }
                i_position_control.update_control_data(downstream_control_data);
                printf("torque command %d milli-Nm\n", downstream_control_data.torque_cmd);
                break;

        //pid coefficients
        case 'k':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                case 'p': //position
                        switch(mode_3)
                        {
                        case 'p':
                                pos_velocity_ctrl_config.P_pos = value;
                                break;
                        case 'i':
                                pos_velocity_ctrl_config.I_pos = value;
                                break;
                        case 'd':
                                pos_velocity_ctrl_config.D_pos = value;
                                break;
                        case 'l':
                                pos_velocity_ctrl_config.integral_limit_pos = value;
                                break;
                        case 'j':
                                pos_velocity_ctrl_config.j = value;
                                break;
                        default:
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                        pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                        printf("Kp:%d Ki:%d Kd:%d j%d i_lim:%d\n",
                                pos_velocity_ctrl_config.P_pos, pos_velocity_ctrl_config.I_pos, pos_velocity_ctrl_config.D_pos,
                                pos_velocity_ctrl_config.j, pos_velocity_ctrl_config.integral_limit_pos);
                        break;

                case 'v': //velocity
                        switch(mode_3)
                        {
                        case 'p':
                                pos_velocity_ctrl_config.P_velocity = value;
                                break;
                        case 'i':
                                pos_velocity_ctrl_config.I_velocity = value;
                                break;
                        case 'd':
                                pos_velocity_ctrl_config.D_velocity = value;
                                break;
                        case 'l':
                                pos_velocity_ctrl_config.integral_limit_velocity = value;
                                break;
                        default:
                                break;
                        }
                        i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                        pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                        printf("Kp:%d Ki:%d Kd:%d i_lim:%d\n", pos_velocity_ctrl_config.P_velocity, pos_velocity_ctrl_config.I_velocity,
                                pos_velocity_ctrl_config.D_velocity, pos_velocity_ctrl_config.integral_limit_velocity);
                        break;

                default:
                        printf("kp->pos_ctrl ko->optimum_ctrl kv->vel_ctrl\n");
                        break;
                }

                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                break;

        //limits
        case 'L':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                //max position limit
                case 'p':
                    switch(mode_3)
                    {
                    case 'u':
                        pos_velocity_ctrl_config.max_pos = value;
                        break;
                    case 'l':
                        pos_velocity_ctrl_config.min_pos = value;
                        break;
                    default:
                        pos_velocity_ctrl_config.max_pos = value;
                        pos_velocity_ctrl_config.min_pos = -value;
                        break;
                    }
                    break;

                //max velocity limit
                case 'v':
                        pos_velocity_ctrl_config.max_speed = value;
                        break;

                //max torque limit
                case 't':
                        pos_velocity_ctrl_config.max_torque = value;
                        break;

                default:
                        break;
                }
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("pos_max:%d pos_min:%d v_max:%d torq_max:%d\n", pos_velocity_ctrl_config.max_pos, pos_velocity_ctrl_config.min_pos, pos_velocity_ctrl_config.max_speed,
                        pos_velocity_ctrl_config.max_torque);
                break;


        //help
        case 'h':
                printf("p->set position\n");
                printf("v->set veloctiy\n");
                printf("k->set PIDs\n");
                printf("L->set limits\n");
                printf("e->enable controllers\n");
                break;

        //jerk limitation (profiler parameters)
        case 'j':
                pos_velocity_ctrl_config = i_position_control.get_position_velocity_control_config();
                switch(mode_2)
                {
                case 'a':
                        pos_velocity_ctrl_config.max_acceleration_profiler = value;
                        break;
                case 'v':
                        pos_velocity_ctrl_config.max_speed_profiler = value;
                        break;
                default:
                        break;
                }
                i_position_control.set_position_velocity_control_config(pos_velocity_ctrl_config);
                printf("acceleration_max:%d velocity_max:%d\n",pos_velocity_ctrl_config.max_acceleration_profiler, pos_velocity_ctrl_config.max_speed_profiler);
                break;

        //set offset
        case 'o':
                motorcontrol_config = i_position_control.get_motorcontrol_config();
                switch(mode_2)
                {
                //set offset
                case 's':
                    motorcontrol_config.commutation_angle_offset = value;
                    i_position_control.set_motorcontrol_config(motorcontrol_config);
                    printf("set offset to %d\n", motorcontrol_config.commutation_angle_offset);
                    break;
                //print offset
                case 'p':
                    printf("offset %d\n", motorcontrol_config.commutation_angle_offset);
                    break;
                }
                break;

        //disable controllers
//        default:
//                i_position_control.disable();
//                printf("controller disabled\n");
//                break;

        }
    }
}

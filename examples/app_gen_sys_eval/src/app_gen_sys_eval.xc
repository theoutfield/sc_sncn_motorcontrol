/*
 * app_gen_sys_eval.xc
 *
 *  Created on: May 9, 2017
 *      Author: sasa
 */
#include <stdio.h>
#include <timer.h>
#include <user_interface_service.h>
#include <xscope.h>
#include <refclk.h>
#include <xs1.h>
#include <dsp.h>

#define MAX_PERCENTAGE 100
#define MIN_PERCENTAGE 0
#define FILTER 100

enum
{
    A,
    B,
    C,
    NR_PHASES
};

int32_t v_dc[FILTER];

void app_gen_sys_eval(client interface TorqueControlInterface i_torque_control)
{
    UpstreamControlData upstream_control_data;
    int phase_voltage_percentage[NR_PHASES] = {0, 0, 0};
    int filter_out = 0;
    float Vdc = 0, V[NR_PHASES] = { 0 }, I[NR_PHASES] = { 0 }, R[NR_PHASES];

    int i = 0;
    int start = 0;
    int nr_measur = 0;
    float rb_rc = 0, ib_ic = 0, ib = 0, vb_va = 0;

    i_torque_control.set_torque_control_disabled();
    i_torque_control.start_system_eval();

    timer tmr;
    unsigned timeout;
    tmr :> timeout;

    printf("app gen syst eval started\n");

    while(1)
    {
        select
            {
            case tmr when timerafter(timeout) :> timeout:
                for (int i= 0; i < NR_PHASES; i++)
                {
                    if (phase_voltage_percentage[i] > MAX_PERCENTAGE)
                        phase_voltage_percentage[i] = MAX_PERCENTAGE;

                    if (phase_voltage_percentage[i] < MIN_PERCENTAGE)
                        phase_voltage_percentage[i] = MIN_PERCENTAGE;

                    if(I[i] < 0)
                        I[i] = -I[i];
                }

                timeout+=1000*MSEC_STD;
                ++nr_measur;

                switch (nr_measur)
                {
                    case 1:
                        phase_voltage_percentage[A] = 30;
                        phase_voltage_percentage[B] = 50;
                        phase_voltage_percentage[C] = 50;
                        break;

                    case 2:
                        phase_voltage_percentage[A] = 50;
                        phase_voltage_percentage[B] = 30;
                        phase_voltage_percentage[C] = 50;
                        rb_rc = I[C]/I[B];  // Rb/Rc
                        ib_ic = I[B] + I[C];
                        ib = I[B];
                        vb_va = V[B] - V[A];
                        break;

                    case 3:
                        R[C] = (V[A] - V[B]) / (I[B] * rb_rc + I[C]);
                        R[B] = rb_rc * R[C];
                        R[A] = (vb_va - ib*R[B])/ib_ic;

                        printf("Ra= %.2f\n", R[A]);
                        printf("Rb= %.2f\n", R[B]);
                        printf("Rc= %.2f\n", R[C]);

                        break;
                }

                i_torque_control.set_evaluation_references(phase_voltage_percentage[A], phase_voltage_percentage[B], phase_voltage_percentage[C]);
                break;

            default:
                upstream_control_data = i_torque_control.update_upstream_control_data();

                /*
                 * moving average filter for DC bus voltage
                 * average of last FILTER values
                 */
                if(i < FILTER)
                    v_dc[i++] = upstream_control_data.V_dc;
                else
                {
                    start = 1;
                    i = 0;
                }

                filter_out = dsp_vector_mean(v_dc, FILTER, 16);

                if (start)
                {
                    /*
                     * voltage calculation
                     */
                    Vdc = (float)filter_out/(1<<16);
                    for (int i = 0; i <  NR_PHASES; i++)
                        V[i] = (float)phase_voltage_percentage[i]/100*filter_out/(1<<16);

//                    xscope_float(VDC, Vdc);
//                    printf("Vdc = %.2f\n", Vdc);
//                    printf("Va = %.2f\n", V[A]);
//                    printf("Vb = %.2f\n", V[B]);
//                    printf("Vc = %.2f\n", V[C]);

                    /*
                     * current calculation
                     */

                    I[B] = (float)upstream_control_data.I_b/(1<<16);
                    I[C] = (float)upstream_control_data.I_c/(1<<16);
                    I[A] = -(I[B] + I[C]);

//                    printf("Ia = %.2f\n", I[A]);
//                    printf("Ib = %.2f\n", I[B]);
//                    printf("Ic = %.2f\n", I[C]);
                }
                break;
        }
    }
}

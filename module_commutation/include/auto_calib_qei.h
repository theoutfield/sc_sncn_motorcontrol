#pragma once

#include <hall_client.h>
#include <qei_client.h>
#include <commutation_client.h>
#include <refclk.h>
#include <velocity_ctrl_client.h>
#include <internal_config.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <auto_calib_hall.h>
#include <hall_qei.h>

void qei_calibrate(chanend c_commutation, commutation_par & commutation_params,
                   HallConfig & hall_config, qei_par & qei_params,
                   chanend c_hall, chanend c_qei, chanend c_calib); //commutation purpose


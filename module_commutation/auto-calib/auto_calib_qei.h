#ifndef AUTO_CALIB_QEI_H_
#define AUTO_CALIB_QEI_H_

#include <dc_motor_config.h>
#include "hall_client.h"
#include "qei_client.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include <auto_calib_hall.h>
#include "hall_qei.h"

void qei_calibrate(chanend c_signal, chanend c_commutation, commutation_par &commutation_params,\
		hall_par &hall_params, qei_par &qei_params, chanend c_hall, chanend c_qei, chanend c_calib); //commutation purpose


#endif /* AUTO_CALIB_QEI_H_ */

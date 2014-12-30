#pragma once

/* Command definitions for BDC client-server communication */
enum { BDC_CMD_SET_VOLTAGE,
       BDC_CMD_DISABLE_FETS,
       BDC_CMD_ENABLE_FETS,
       BDC_CMD_FETS_STATE,
       BDC_CMD_CHECK_BUSY };

#define PWM_MIN_LIMIT 250 /* FIXME: remove it when proper PWM module is used */

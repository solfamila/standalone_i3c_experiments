/*
 * Local standalone override for experiment 03 master.
 */

#define EXPERIMENT_USE_SMARTDMA 0
#define EZH_ROUNDTRIP_DATA_LENGTH 8U
#define EXPERIMENT_STARTUP_WAIT 5000000U
#define EXPERIMENT_POST_DAA_CLEANUP 0U
#define EXPERIMENT_POST_DAA_WAIT 50000000U
#define EXPERIMENT_I3C_PP_BAUDRATE 4000000U

#include "../../common/ezh_test_roundtrip.c"

#include "states.h"

State safe;
State armed;
State boost1;
State fast1;
State pre_stage;
State post_stage;
State sustainer_ignition;
State boost2;
State fast2;
State apogee;
State coast;
State chute;
State landed;

StateMachine flight_state_machine;

extern void safe_initialize(double currentTimestampSec);
extern State *safe_execute(double currentTimestampSec);
extern void safe_finish(double currentTimestampSec);

extern void armed_initialize(double currentTimestampSec);
extern State *armed_execute(double currentTimestampSec);
extern void armed_finish(double currentTimestampSec);

extern void boost1_initialize(double currentTimestampSec);
extern State *boost1_execute(double currentTimestampSec);
extern void boost1_finish(double currentTimestampSec);

extern void fast1_initialize(double currentTimestampSec);
extern State *fast1_execute(double currentTimestampSec);
extern void fast1_finish(double currentTimestampSec);

extern void pre_stage_initialize(double currentTimestampSec);
extern State *pre_stage_execute(double currentTimestampSec);
extern void pre_stage_finish(double currentTimestampSec);

extern void post_stage_initialize(double currentTimestampSec);
extern State *post_stage_execute(double currentTimestampSec);
extern void post_stage_finish(double currentTimestampSec);

extern void sustainer_ignition_initialize(double currentTimestampSec);
extern State *sustainer_ignition_execute(double currentTimestampSec);
extern void sustainer_ignition_finish(double currentTimestampSec);

extern void boost2_initialize(double currentTimestampSec);
extern State *boost2_execute(double currentTimestampSec);
extern void boost2_finish(double currentTimestampSec);

extern void fast2_initialize(double currentTimestampSec);
extern State *fast2_execute(double currentTimestampSec);
extern void fast2_finish(double currentTimestampSec);

extern void apogee_initialize(double currentTimestampSec);
extern State *apogee_execute(double currentTimestampSec);
extern void apogee_finish(double currentTimestampSec);

extern void coast_initialize(double currentTimestampSec);
extern State *coast_execute(double currentTimestampSec);
extern void coast_finish(double currentTimestampSec);

extern void chute_initialize(double currentTimestampSec);
extern State *chute_execute(double currentTimestampSec);
extern void chute_finish(double currentTimestampSec);

extern void landed_initialize(double currentTimestampSec);
extern State *landed_execute(double currentTimestampSec);
extern void landed_finish(double currentTimestampSec);

void init_flight_state_machine() {
    INIT_STATE(safe, safe_initialize, safe_execute, safe_finish);
    INIT_STATE(armed, armed_initialize, armed_execute, armed_finish);
    INIT_STATE(boost1, boost1_initialize, boost1_execute, boost1_finish);
    INIT_STATE(fast1, fast1_initialize, fast1_execute, fast1_finish);
    INIT_STATE(pre_stage, pre_stage_initialize, pre_stage_execute, pre_stage_finish);
    INIT_STATE(post_stage, post_stage_initialize, post_stage_execute, post_stage_finish);
    INIT_STATE(sustainer_ignition, sustainer_ignition_initialize, sustainer_ignition_execute, sustainer_ignition_finish);
    INIT_STATE(boost2, boost2_initialize, boost2_execute, boost2_finish);
    INIT_STATE(fast2, fast2_initialize, fast2_execute, fast2_finish);
    INIT_STATE(apogee, apogee_initialize, apogee_execute, apogee_finish);
    INIT_STATE(coast, coast_initialize, coast_execute, coast_finish);
    INIT_STATE(chute, chute_initialize, chute_execute, chute_finish);
    INIT_STATE(landed, landed_initialize, landed_execute, landed_finish);

    init_state_machine(&flight_state_machine, &safe);
}
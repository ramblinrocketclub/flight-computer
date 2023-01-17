#include "states/states.h"

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

extern void safe_initialize();
extern State *safe_execute();
extern void safe_finish();

extern void armed_initialize();
extern State *armed_execute();
extern void armed_finish();

extern void boost1_initialize();
extern State *boost1_execute();
extern void boost1_finish();

extern void fast1_initialize();
extern State *fast1_execute();
extern void fast1_finish();

extern void pre_stage_initialize();
extern State *pre_stage_execute();
extern void pre_stage_finish();

extern void post_stage_initialize();
extern State *post_stage_execute();
extern void post_stage_finish();

extern void sustainer_ignition_initialize();
extern State *sustainer_ignition_execute();
extern void sustainer_ignition_finish();

extern void boost2_initialize();
extern State *boost2_execute();
extern void boost2_finish();

extern void fast2_initialize();
extern State *fast2_execute();
extern void fast2_finish();

extern void apogee_initialize();
extern State *apogee_execute();
extern void apogee_finish();

extern void coast_initialize();
extern State *coast_execute();
extern void coast_finish();

extern void chute_initialize();
extern State *chute_execute();
extern void chute_finish();

extern void landed_initialize();
extern State *landed_execute();
extern void landed_finish();

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
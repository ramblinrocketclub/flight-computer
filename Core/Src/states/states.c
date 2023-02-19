#include "states.h"

State safe;
State armed;
State boost1;
State coast1;
State boost2;
State coast2;
State recovery;
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

extern void coast1_initialize(double currentTimestampSec);
extern State *coast1_execute(double currentTimestampSec);
extern void coast1_finish(double currentTimestampSec);

extern void boost2_initialize(double currentTimestampSec);
extern State *boost2_execute(double currentTimestampSec);
extern void boost2_finish(double currentTimestampSec);

extern void coast2_initialize(double currentTimestampSec);
extern State *coast2_execute(double currentTimestampSec);
extern void coast2_finish(double currentTimestampSec);

extern void recovery_initialize(double currentTimestampSec);
extern State *recovery_execute(double currentTimestampSec);
extern void recovery_finish(double currentTimestampSec);

extern void landed_initialize(double currentTimestampSec);
extern State *landed_execute(double currentTimestampSec);
extern void landed_finish(double currentTimestampSec);

void init_flight_state_machine() {
    INIT_STATE(safe, safe_initialize, safe_execute, safe_finish);
    INIT_STATE(armed, armed_initialize, armed_execute, armed_finish);
    INIT_STATE(boost1, boost1_initialize, boost1_execute, boost1_finish);
    INIT_STATE(coast1, coast1_initialize, coast1_execute, coast1_finish);
    INIT_STATE(boost2, boost2_initialize, boost2_execute, boost2_finish);
    INIT_STATE(coast2, coast2_initialize, coast2_execute, coast2_finish);
    INIT_STATE(recovery, recovery_initialize, recovery_execute, recovery_finish);
    INIT_STATE(landed, landed_initialize, landed_execute, landed_finish);

    init_state_machine(&flight_state_machine, &safe);
}
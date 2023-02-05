#ifndef POST_STAGE_H
#define POST_STAGE_H

#include "state_machine.h"

void post_stage_initialize(double currentTimestampSec);
State *post_stage_execute(double currentTimestampSec);
void post_stage_finish(double currentTimestampSec);

#endif /* POST_STAGE_H */
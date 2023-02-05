#ifndef PRE_STAGE_H
#define PRE_STAGE_H

#include "state_machine.h"

void pre_stage_initialize(double currentTimestampSec);
State *pre_stage_execute(double currentTimestampSec);
void pre_stage_finish(double currentTimestampSec);

#endif /* PRE_STAGE_H */
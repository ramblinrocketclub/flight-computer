#ifndef PRE_STAGE_H
#define PRE_STAGE_H

#include "state_machine.h"

void pre_stage_initialize();
State *pre_stage_execute();
void pre_stage_finish();

#endif /* PRE_STAGE_H */
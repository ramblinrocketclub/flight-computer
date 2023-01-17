#ifndef POST_STAGE_H
#define POST_STAGE_H

#include "state_machine.h"

void post_stage_initialize();
State *post_stage_execute();
void post_stage_finish();

#endif /* POST_STAGE_H */
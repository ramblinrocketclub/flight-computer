#ifndef CHUTE_H
#define CHUTE_H

#include "state_machine.h"

void chute_initialize(double currentTimestampSec);
State *chute_execute(double currentTimestampSec);
void chute_finish(double currentTimestampSec);

#endif /* CHUTE_H */
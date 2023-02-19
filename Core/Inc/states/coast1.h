#ifndef COAST1_H
#define COAST1_H

#include "state_machine.h"

void coast1_initialize(double currentTimestampSec);
State *coast1_execute(double currentTimestampSec);
void coast1_finish(double currentTimestampSec);

#endif /* COAST1_H */
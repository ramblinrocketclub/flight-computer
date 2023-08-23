#ifndef COAST2_H
#define COAST2_H

#include "state_machine.h"

void coast2_initialize(double currentTimestampSec);
State *coast2_execute(double currentTimestampSec);
void coast2_finish(double currentTimestampSec);

#endif /* COAST2_H */
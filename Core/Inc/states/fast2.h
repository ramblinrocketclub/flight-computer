#ifndef FAST2_H
#define FAST2_H

#include "state_machine.h"

void fast2_initialize(double currentTimestampSec);
State *fast2_execute(double currentTimestampSec);
void fast2_finish(double currentTimestampSec);

#endif /* FAST2_H */
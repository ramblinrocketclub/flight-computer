#ifndef SAFE_H
#define SAFE_H

#include "state_machine.h"

void safe_initialize(double currentTimestampSec);
State *safe_execute(double currentTimestampSec);
void safe_finish(double currentTimestampSec);

#endif /* SAFE_H */
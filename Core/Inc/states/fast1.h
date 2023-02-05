#ifndef FAST1_H
#define FAST1_H

#include "state_machine.h"

void fast1_initialize(double currentTimestampSec);
State *fast1_execute(double currentTimestampSec);
void fast1_finish(double currentTimestampSec);

#endif /* FAST1_H */
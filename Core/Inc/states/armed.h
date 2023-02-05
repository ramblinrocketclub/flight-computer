#ifndef ARMED_H
#define ARMED_H

#include "state_machine.h"

void armed_initialize(double currentTimestampSec);
State *armed_execute(double currentTimestampSec);
void armed_finish(double currentTimestampSec);

#endif /* ARMED_H */
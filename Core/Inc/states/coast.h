#ifndef COAST_H
#define COAST_H

#include "state_machine.h"

void coast_initialize(double currentTimestampSec);
State *coast_execute(double currentTimestampSec);
void coast_finish(double currentTimestampSec);

#endif /* COAST_H */
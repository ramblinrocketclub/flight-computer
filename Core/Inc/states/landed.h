#ifndef LANDED_H
#define LANDED_H

#include "state_machine.h"

void landed_initialize(double currentTimestampSec);
State *landed_execute(double currentTimestampSec);
void landed_finish(double currentTimestampSec);

#endif /* LANDED_H */
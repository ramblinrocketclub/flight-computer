#ifndef RECOVERY_H
#define RECOVERY_H

#include "state_machine.h"

void recovery_initialize(double currentTimestampSec);
State *recovery_execute(double currentTimestampSec);
void recovery_finish(double currentTimestampSec);

#endif /* RECOVERY_H */
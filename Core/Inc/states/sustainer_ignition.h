#ifndef SUSTAINER_IGNITION_H
#define SUSTAINER_IGNITION_H

#include "state_machine.h"

void sustainer_ignition_initialize(double currentTimestampSec);
State *sustainer_ignition_execute(double currentTimestampSec);
void sustainer_ignition_finish(double currentTimestampSec);

#endif /* SUSTAINER_IGNITION_H */
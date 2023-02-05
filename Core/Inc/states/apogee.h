#ifndef APOGEE_H
#define APOGEE_H

#include "state_machine.h"

void apogee_initialize(double currentTimestampSec);
State *apogee_execute(double currentTimestampSec);
void apogee_finish(double currentTimestampSec);

#endif /* APOGEE_H */
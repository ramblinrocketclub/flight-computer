#ifndef BOOST1_H
#define BOOST1_H

#include "state_machine.h"

void boost1_initialize(double currentTimestampSec);
State *boost1_execute(double currentTimestampSec);
void boost1_finish(double currentTimestampSec);

#endif /* BOOST1_H */
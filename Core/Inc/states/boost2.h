#ifndef BOOST2_H
#define BOOST2_H

#include "state_machine.h"

void boost2_initialize(double currentTimestampSec);
State *boost2_execute(double currentTimestampSec);
void boost2_finish(double currentTimestampSec);

#endif /* BOOST2_H */
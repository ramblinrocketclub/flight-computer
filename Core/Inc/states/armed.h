#ifndef ARMED_H
#define ARMED_H

#include "state_machine.h"

void armed_initialize();
State *armed_execute();
void armed_finish();

#endif /* ARMED_H */
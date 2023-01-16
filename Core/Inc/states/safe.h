#ifndef SAFE_H
#define SAFE_H

#include "state_machine.h"

void safe_initialize();
State *safe_execute();
void safe_finish();

#endif /* SAFE_H */
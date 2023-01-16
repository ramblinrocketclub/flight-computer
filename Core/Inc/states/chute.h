#ifndef CHUTE_H
#define CHUTE_H

#include "state_machine.h"

void chute_initialize();
State *chute_execute();
void chute_finish();

#endif /* CHUTE_H */
#ifndef COAST_H
#define COAST_H

#include "state_machine.h"

void coast_initialize();
State *coast_execute();
void coast_finish();

#endif /* COAST_H */
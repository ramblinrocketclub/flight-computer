#ifndef LANDED_H
#define LANDED_H

#include "state_machine.h"

void landed_initialize();
State *landed_execute();
void landed_finish();

#endif /* LANDED_H */
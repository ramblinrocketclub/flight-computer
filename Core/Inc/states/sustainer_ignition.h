#ifndef SUSTAINER_IGNITION_H
#define SUSTAINER_IGNITION_H

#include "state_machine.h"

void sustainer_ignition_initialize();
State *sustainer_ignition_execute();
void sustainer_ignition_finish();

#endif /* SUSTAINER_IGNITION_H */
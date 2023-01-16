#ifndef APOGEE_H
#define APOGEE_H

#include "state_machine.h"

void apogee_initialize();
State *apogee_execute();
void apogee_finish();

#endif /* APOGEE_H */
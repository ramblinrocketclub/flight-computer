#ifndef ROCKET_H
#define ROCKET_H

#include "state_machine.h"

// Declare the private instance of Rocket state machine
SM_DECLARE(RKT_StateMachine);

// State machine event functions
EVENT_DECLARE(RKT_Start, NoEventData);

#endif /* ROCKET_H */


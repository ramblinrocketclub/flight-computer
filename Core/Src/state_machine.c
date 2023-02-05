#include "state_machine.h"

#include <stddef.h>
#include "printf.h"

void init_state_machine(StateMachine *sm, State *initialState) {
    sm->currentState = initialState;

    if (sm->currentState == NULL) {
        printf("Initial state for state machine must not be null!\n");
    }

    sm->hasFirstStateInitialized = false;
}

void step_state_machine(StateMachine *sm, double currentTimestampSec) {
    if (!sm->hasFirstStateInitialized) {
        (*sm->currentState->initPtr)(currentTimestampSec);
        sm->hasFirstStateInitialized = true;
        return;
    }

    State *nextState = (*sm->currentState->executePtr)(currentTimestampSec);

    if (nextState != NULL) {
        if (nextState != sm->currentState) {
            (*sm->currentState->finishPtr)(currentTimestampSec);
            sm->currentState = nextState;
            (*sm->currentState->initPtr)(currentTimestampSec);
        }
    } else {
        printf("State machine has effectively terminated due to a null state\n");
    }
}
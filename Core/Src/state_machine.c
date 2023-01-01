#include "state_machine.h"

#include <stddef.h>
#include "log.h"

void init_state_machine(StateMachine *sm, State *initialState) {
    sm->currentState = initialState;

    if (sm->currentState == NULL) {
        LOG_FATAL("Initial state for state machine must not be null!");
    }

    sm->hasFirstStateInitialized = false;
}

void step_state_machine(StateMachine *sm) {
    if (!sm->hasFirstStateInitialized) {
        (*sm->currentState->initPtr)();
        sm->hasFirstStateInitialized = true;
        return;
    }

    State *nextState = (*sm->currentState->executePtr)();

    if (nextState != NULL) {
        if (nextState != sm->currentState) {
            (*sm->currentState->finishPtr)();
            sm->currentState = nextState;
            (*sm->currentState->initPtr)();
        }
    } else {
        LOG_WARN("State machine has effectively terminated due to a null state");
    }
}
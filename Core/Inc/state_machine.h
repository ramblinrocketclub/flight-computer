#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct State {
    // The name of the state (for debugging purposes)
    char *name;

    // All state function pointers take in a timestamp as a double
    // The init function pointer is invoked when the state is entered
    void (*initPtr)(double);

    // The execute function pointer is invoked repeatedly once the
    // state is entered and returns the next state to execute
    struct State* (*executePtr)(double);

    // The finish function pointer is invoked when the state machine
    // transitions to a new state
    void (*finishPtr)(double);
} State;

typedef struct StateMachine {
    bool hasFirstStateInitialized;

    State *currentState;
} StateMachine;

void init_state_machine(StateMachine *sm, State *initialState);
void step_state_machine(StateMachine *sm, double currentTimestampSec);

#endif /* STATE_MACHINE_H */
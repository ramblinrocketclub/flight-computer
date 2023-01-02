#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct State {
    // The name of the state (for debugging purposes)
    char *name;

    // The init function pointer is invoked when the state is entered
    void (*initPtr)();

    // The execute function pointer is invoked repeatedly once the
    // state is entered and returns the next state to execute
    struct State* (*executePtr)();

    // The finish function pointer is invoked when the state machine
    // transitions to a new state
    void (*finishPtr)();
} State;

typedef struct StateMachine {
    bool hasFirstStateInitialized;

    State *currentState;
} StateMachine;

void init_state_machine(StateMachine *sm, State *initialState);
void step_state_machine(StateMachine *sm);

#endif /* STATE_MACHINE_H */
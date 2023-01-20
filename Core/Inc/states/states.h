#ifndef STATES_H
#define STATES_H

#include "state_machine.h"
#include "flight_state_variables.h"
#include "event_constants.h"

#define INIT_STATE(state, init_fn, execute_fn, finish_fn) { \
                            state.initPtr = &init_fn;       \
                            state.executePtr = &execute_fn; \
                            state.finishPtr = &finish_fn;   \
                        }

// Flight states
extern State safe;
extern State armed;
extern State boost1;
extern State fast1;
extern State pre_stage;
extern State post_stage;
extern State sustainer_ignition;
extern State boost2;
extern State fast2;
extern State apogee;
extern State coast;
extern State chute;
extern State landed;

extern StateMachine flight_state_machine;

extern FlightStateVariables state_variables;

void init_flight_state_machine();

#endif /* STATES_H */
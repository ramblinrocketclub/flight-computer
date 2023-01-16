#include <stddef.h>
#include <stdint.h>
#include "state_machine.h"

void _SM_ExternalEvent(SM_StateMachine* self, const SM_StateMachineConst* selfConst, uint8_t newState, void *pEventData)
{
    if (newState == EVENT_IGNORED)
    {
        return;
    }

    // TODO - capture software lock here for thread-safety if necessary

    // Generate the event
    _SM_InternalEvent(self, newState, pEventData);

    // Execute state machine based on type of state map defined
    if (selfConst->stateMap)
    {
        _SM_StateEngine(self, selfConst);
    }

    else
    {
        _SM_StateEngineEx(self, selfConst);
    }

    // TODO - release software lock here

}

void _SM_InternalEvent(SM_StateMachine* self, uint8_t newState, void *pEventData)
{
    self->pEventData = pEventData;
    self->eventGenerated = true;
    self->newState = newState;

}

void _SM_StateEngine(SM_StateMachine* self, const SM_StateMachineConst* selfConst)
{
    void* pDataTemp = NULL;

    // While events are being generated keep executing states
    while (self->eventGenerated)
    {
        // Get the pointers from the state map
        SM_StateFunc state = selfConst->stateMap[self->newState].pStateFunc;

        // Copy of event data pointer
        pDataTemp = self->pEventData;

        // Event data used up, reset the pointer
        self->pEventData = NULL;

        // Event used up, reset the flag
        self->eventGenerated = false;

        // Switch to the new current state
        self->currentState = self->newState;

        // Execute the state action passing in event data
        state(self, pDataTemp);
    }
}

void _SM_StateEngineEx(SM_StateMachine* self, const SM_StateMachineConst* selfConst)
{
    bool guardResult = true;
    void* pDataTemp = NULL;

    // While events are being generated keep executing states
    while (self->eventGenerated)
    {
        // Get the pointers from the extended state map
        SM_StateFunc state = selfConst->stateMapEx[self->newState].pStateFunc;
        SM_GuardFunc guard = selfConst->stateMapEx[self->newState].pGuardFunc;
        SM_EntryFunc entry = selfConst->stateMapEx[self->newState].pEntryFunc;
        SM_ExitFunc exit = selfConst->stateMapEx[self->currentState].pExitFunc;

        // Copy of event data pointer
        pDataTemp = self->pEventData;

        // Event data used up, reset the pointer
        self->pEventData = NULL;

        // Event used up, reset the flag
        self->eventGenerated = false;

        // Execute the guard condition
        if (guard != NULL)
        {
            guardResult = guard(self, pDataTemp);
        }

        // If the guard condition succeeds
        if (guardResult == true)
        {
            // Transitioning to a new state?
            if (self->newState != self->currentState)
            {
                // Execute the state exit action on current state before switching to new state
                if (exit != NULL)
                {
                    exit(self);
                }

                // Execute the state entry action on the new state
                if (entry != NULL)
                {
                    entry(self, pDataTemp);
                }
            }

            // Switch to the new current state
            self->currentState = self->newState;

            // Execute the state action passing in event data
            state(self, pDataTemp);
        }
    }
}


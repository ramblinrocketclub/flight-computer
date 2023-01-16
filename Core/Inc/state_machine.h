#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>

enum { EVENT_IGNORED = 0xFE, CANNOT_HAPPEN = 0xFF };

typedef void NoEventData;

// State machine constant data
typedef struct
{
    const char* name;
    const uint8_t maxStates;
    const struct SM_StateStruct* stateMap;
    const struct SM_StateStructEx* stateMapEx;
} SM_StateMachineConst;

// State machine instance data
typedef struct
{
    const char* name;
    void* pInstance;
    uint8_t newState;
    uint8_t currentState;
    bool eventGenerated;
    void* pEventData;
} SM_StateMachine;

// Generic state function signatures
typedef void (*SM_StateFunc)(SM_StateMachine* self, void* pEventData);
typedef bool (*SM_GuardFunc)(SM_StateMachine* self, void* pEventData);
typedef void (*SM_EntryFunc)(SM_StateMachine* self, void* pEventData);
typedef void (*SM_ExitFunc)(SM_StateMachine* self);

typedef struct SM_StateStruct
{
    SM_StateFunc pStateFunc;
} SM_StateStruct;

typedef struct SM_StateStructEx
{
    SM_StateFunc pStateFunc;
    SM_GuardFunc pGuardFunc;
    SM_EntryFunc pEntryFunc;
    SM_ExitFunc pExitFunc;
} SM_StateStructEx;

// Public functions
#define SM_Event(_smName_, _eventFunc_, _eventData_) \
    _eventFunc_(&_smName_##Obj, _eventData_)
#define SM_Get(_smName_, _getFunc_) \
    _getFunc_(&_smName_##Obj)

// Protected functions
#define SM_InternalEvent(_newState_, _eventData_) \
    _SM_InternalEvent(self, _newState_, _eventData_)
#define SM_GetInstance(_instance_) \
    (_instance_*)(self->pInstance);

// Private functions
void _SM_ExternalEvent(SM_StateMachine* self, const SM_StateMachineConst* selfConst, uint8_t newState, void* pEventData);
void _SM_InternalEvent(SM_StateMachine* self, uint8_t newState, void* pEventData);
void _SM_StateEngine(SM_StateMachine* self, const SM_StateMachineConst* selfConst);
void _SM_StateEngineEx(SM_StateMachine* self, const SM_StateMachineConst* selfConst);

#define SM_DECLARE(_smName_) \
    extern SM_StateMachine _smName_##Obj;

#define SM_DEFINE(_smName_, _instance_) \
    SM_StateMachine _smName_##Obj = { #_smName_, _instance_, \
        0, 0, 0, 0 };

#define EVENT_DECLARE(_eventFunc_, _eventData_) \
    void _eventFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define EVENT_DEFINE(_eventFunc_, _eventData_) \
    void _eventFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define GET_DECLARE(_getFunc_, _getData_) \
    _getData_ _getFunc_(SM_StateMachine* self);

#define GET_DEFINE(_getFunc_, _getData_) \
    _getData_ _getFunc_(SM_StateMachine* self)

#define STATE_DECLARE(_stateFunc_, _eventData_) \
    static void ST_##_stateFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define STATE_DEFINE(_stateFunc_, _eventData_) \
    static void ST_##_stateFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define GUARD_DECLARE(_guardFunc_, _eventData_) \
    static bool GD_##_guardFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define GUARD_DEFINE(_guardFunc_, _eventData_) \
    static bool GD_##_guardFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define ENTRY_DECLARE(_entryFunc_, _eventData_) \
    static void EN_##_entryFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define ENTRY_DEFINE(_entryFunc_, _eventData_) \
    static void EN_##_entryFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define EXIT_DECLARE(_exitFunc_) \
    static void EX_##_exitFunc_(SM_StateMachine* self);

#define EXIT_DEFINE(_exitFunc_) \
    static void EX_##_exitFunc_(SM_StateMachine* self)

#define BEGIN_STATE_MAP(_smName_) \
    static const SM_StateStruct _smName_##StateMap[] = {

#define STATE_MAP_ENTRY(_stateFunc_) \
    { (SM_StateFunc)_stateFunc_ },

#define END_STATE_MAP(_smName_) \
    }; \
    static const SM_StateMachineConst _smName_##Const = { #_smName_, \
        (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])), \
        _smName_##StateMap, NULL };

#define BEGIN_STATE_MAP_EX(_smName_) \
    static const SM_StateStructEx _smName_##StateMap[] = {

#define STATE_MAP_ENTRY_EX(_stateFunc_) \
    { (SM_StateFunc)_stateFunc_, NULL, NULL, NULL },

#define STATE_MAP_ENTRY_ALL_EX(_stateFunc_, _guardFunc_, _entryFunc_, _exitFunc_) \
    { (SM_StateFunc)_stateFunc_, (SM_GuardFunc)_guardFunc_, (SM_EntryFunc)_entryFunc_, (SM_ExitFunc)_exitFunc_ },

#define END_STATE_MAP_EX(_smName_) \
    }; \
    static const SM_StateMachineConst _smName_##Const = { #_smName_, \
        (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])), \
        NULL, _smName_##StateMap };

#define BEGIN_TRANSITION_MAP \
    static const uint8_t TRANSITIONS[] = { \

#define TRANSITION_MAP_ENTRY(_entry_) \
    _entry_,

#define END_TRANSITION_MAP(_smName_, _eventData_) \
    }; \
    _SM_ExternalEvent(self, &_smName_##Const, TRANSITIONS[self->currentState], _eventData_); \
    C_ASSERT((sizeof(TRANSITIONS)/sizeof(uint8_t)) == (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])));

#endif // STATE_MACHINE_H


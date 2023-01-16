#include <stdint.h>
#include "state_machine.h"
#include "rocket.h"

// Rocket object structure
typedef struct
{
    double  linear_position_x;
    double  linear_position_y;
    double  linear_position_z;

    double  linear_velocity_x;
    double  linear_velocity_y;
    double  linear_velocity_z;

    double  linear_acceleration_x;
    double  linear_acceleration_y;
    double  linear_acceleration_z;
} RKT_t;

// Define private instance of Rocket state machine
RKT_t rocket;
SM_DEFINE(RKT_StateMachine, &rocket);

enum States
{
    ST_IDLE,
    ST_SAFE,
    ST_ARMED,
    ST_BOOST_1,
    ST_FAST_1,
    ST_PRE_STAGE,
    ST_STAGE,
    ST_FAILED_STAGING,
    ST_POST_STAGE,
    ST_SUSTAINER_IGNITION,
    ST_FAILED_SUSTAINER_IGNITION,
    ST_BOOST_2,
    ST_FAST_2,
    ST_APOGEE,
    ST_COAST,
    ST_CHUTE,
    ST_LANDED
};

// State machine state functions
STATE_DECLARE(Idle,                     NoEventData);
STATE_DECLARE(Safe,                     NoEventData);
STATE_DECLARE(Armed,                    NoEventData);
STATE_DECLARE(Boost1,                   NoEventData);
STATE_DECLARE(Fast1,                    NoEventData);
STATE_DECLARE(PreStage,                 NoEventData);
STATE_DECLARE(Stage,                    NoEventData);
STATE_DECLARE(FailedStaging,            NoEventData);
STATE_DECLARE(PostStage,                NoEventData);
STATE_DECLARE(SustainerIgnition,        NoEventData);
STATE_DECLARE(FailedSustainerIgnition,  NoEventData);
STATE_DECLARE(Boost2,                   NoEventData);
STATE_DECLARE(Fast2,                    NoEventData);
STATE_DECLARE(Apogee,                   NoEventData);
STATE_DECLARE(Chute,                    NoEventData);
STATE_DECLARE(Landed,                   NoEventData);

STATE_DEFINE(Idle,                     NoEventData)
{

}

STATE_DEFINE(Safe,                     NoEventData)
{

}

STATE_DEFINE(Armed,                    NoEventData)
{

}

STATE_DEFINE(Boost1,                   NoEventData)
{

}

STATE_DEFINE(Fast1,                    NoEventData)
{

}

STATE_DEFINE(PreStage,                 NoEventData)
{

}

STATE_DEFINE(Stage,                    NoEventData)
{

}

STATE_DEFINE(FailedStaging,            NoEventData)
{

}

STATE_DEFINE(PostStage,                NoEventData)
{

}

STATE_DEFINE(SustainerIgnition,        NoEventData)
{

}

STATE_DEFINE(FailedSustainerIgnition,  NoEventData)
{

}

STATE_DEFINE(Boost2,                   NoEventData)
{

}

STATE_DEFINE(Fast2,                    NoEventData)
{

}

STATE_DEFINE(Apogee,                   NoEventData)
{

}

STATE_DEFINE(Chute,                    NoEventData)
{

}

STATE_DEFINE(Landed,                   NoEventData)
{

}


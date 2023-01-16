#ifndef ROCKET_H
#define ROCKET_H

#include "state_machine.h"

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

// Declare the private instance of Rocket state machine
SM_DECLARE(RKT_StateMachine);

// State machine event functions
EVENT_DECLARE(RKT_Start, NoEventData);

#endif /* ROCKET_H */


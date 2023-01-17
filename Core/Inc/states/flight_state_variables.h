#ifndef FLIGHT_STATE_VARIABLES_H
#define FLIGHT_STATE_VARIABLES_H

// Contains all of the information needed to make flight decisions
typedef struct FlightStateVariables {
    double verticalAcceleration;
    double verticalVelocity;
    double verticalPosition; // This position is wrt starting launch height
    double tiltRadians;
} FlightStateVariables;

#endif /* FLIGHT_STATE_VARIABLES_H */
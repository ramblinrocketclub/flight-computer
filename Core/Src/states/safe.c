#include "safe.h"

#include "states.h"

void safe_initialize(double currentTimestampSec) {
    printf("Entered safe state");
}

State *safe_execute(double currentTimestampSec) {
    bool ematches_connected = false;
    bool pull_switches_armed = false;

    // Launch conditions
    bool gps_lock = false;
    bool battery_voltage_ok = false;
    bool healthy_cameras = false;

    if (ematches_connected && pull_switches_armed
        && gps_lock && battery_voltage_ok && healthy_cameras) {
        return &armed;
    }

    return &safe;
}

void safe_finish(double currentTimestampSec) {
    
}
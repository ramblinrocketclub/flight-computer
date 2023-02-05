#include "sustainer_ignition.h"

#include "states.h"

void sustainer_ignition_initialize(double currentTimestampSec) {

}

State *sustainer_ignition_execute(double currentTimestampSec) {
    return &sustainer_ignition;
}

void sustainer_ignition_finish(double currentTimestampSec) {
    
}
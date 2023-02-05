#include "pre_stage.h"

#include "states.h"

void pre_stage_initialize(double currentTimestampSec) {

}

State *pre_stage_execute(double currentTimestampSec) {
    return &pre_stage;
}

void pre_stage_finish(double currentTimestampSec) {
    
}
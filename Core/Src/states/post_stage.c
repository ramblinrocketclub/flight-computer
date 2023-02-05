#include "post_stage.h"

#include "states.h"

void post_stage_initialize(double currentTimestampSec) {

}

State *post_stage_execute(double currentTimestampSec) {
    return &post_stage;
}

void post_stage_finish(double currentTimestampSec) {
    
}
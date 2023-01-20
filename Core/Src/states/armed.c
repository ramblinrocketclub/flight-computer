#include "armed.h"

#include "states.h"

void armed_initialize() {

}

State *armed_execute() {
    return &armed;
}

void armed_finish() {
    
}
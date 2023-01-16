#include "states/armed.h"

#include "states/states.h"

void armed_initialize() {

}

State *armed_execute() {
    return &armed;
}

void armed_finish() {
    
}
#include "rolling_window.h"

#include "printf.h"

void remove_first(RollingWindow *rw);

void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity) {
    rw->backing_array = pBackingArray;
    rw->capacity = capacity;
    rw->size = 0;
    rw->front = 0;
}

int mod_rolling_window(int index, int modulo) {
    if (modulo <= 0) {
        printf("The modulo must be positive!\n");
    }
    
    int newIndex = index % modulo;
    return newIndex >= 0 ? newIndex : newIndex + modulo;
}

void add_data_point(RollingWindow *rw, double dataPoint) {
    // If we have fully filled the buffer, remove first data point so it can be overwritten
    if (rw->size >= rw->capacity) {
        remove_first(rw);
    }
}

void remove_first(RollingWindow *rw) {
    if (rw->size <= 0) {
        printf("Cannot remove element from rolling window when it is empty\n");
    }
}
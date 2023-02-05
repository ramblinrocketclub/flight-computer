#include <stdint.h>
#include <stddef.h>
#include "rolling_window.h"

#include "printf.h"

void remove_first_rolling_window(RollingWindow *rw);
size_t mod_rolling_window(size_t index, size_t modulo);
size_t get_safe_front_rolling_window(RollingWindow *rw);
size_t get_safe_back_rolling_window(RollingWindow *rw);

void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity) {
    rw->backing_array = pBackingArray;
    rw->capacity = capacity;
    rw->size = 0;
    rw->front = 0;
}

size_t mod_rolling_window(size_t index, size_t modulo) {
    if (modulo <= 0) {
        printf("The modulo must be positive!\n");
    }
    
    size_t newIndex = index % modulo;
    return newIndex >= 0 ? newIndex : newIndex + modulo;
}

size_t get_safe_front_rolling_window(RollingWindow *rw) {
    return mod_rolling_window(rw->front, rw->capacity);
}

size_t get_safe_back_rolling_window(RollingWindow *rw) {
    return mod_rolling_window(rw->front + rw->size - 1, rw->capacity);
}

void add_data_point_rolling_window(RollingWindow *rw, double dataPoint) {
    // If we have fully filled the buffer, remove first data point so it can be overwritten
    if (rw->size >= rw->capacity) {
        remove_first_rolling_window(rw);
    }

    size_t back_insertion_idx = mod_rolling_window(rw->front + rw->size, rw->capacity);

    rw->backing_array[back_insertion_idx] = dataPoint;

    rw->sum_of_elements += dataPoint;

    rw->size++;
}

void remove_first_rolling_window(RollingWindow *rw) {
    if (rw->size <= 0) {
        printf("Cannot remove element from rolling window when it is empty\n");
    }

    rw->sum_of_elements -= rw->backing_array[get_safe_front_rolling_window(rw)];

    rw->backing_array[get_safe_front_rolling_window(rw)] = 0.0;

    rw->front = mod_rolling_window(rw->front + 1, rw->capacity);

    rw->size--;
}

double get_datapoint_at_index_rolling_window(RollingWindow *rw, size_t index) {
    return rw->backing_array[mod_rolling_window(rw->front + index, rw->capacity)];
}

double get_latest_datapoint_rolling_window(RollingWindow *rw) {
    return get_datapoint_at_index_rolling_window(rw, rw->size - 1);
}

double get_earliest_datapoint_rolling_window(RollingWindow *rw) {
    return get_datapoint_at_index_rolling_window(rw, 0);
}
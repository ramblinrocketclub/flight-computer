#ifndef ROLLING_WINDOW_H
#define ROLLING_WINDOW_H

typedef struct RollingWindow {
    double *backing_array;
    size_t capacity;
    size_t size;
    size_t front;
    double sum_of_elements;
} RollingWindow;

void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity);
void add_data_point_rolling_window(RollingWindow *rw, double dataPoint);
double get_datapoint_at_index_rolling_window(RollingWindow *rw, size_t index);
double get_latest_datapoint_rolling_window(RollingWindow *rw);
double get_earliest_datapoint_rolling_window(RollingWindow *rw);

#endif /* ROLLING_WINDOW_H */
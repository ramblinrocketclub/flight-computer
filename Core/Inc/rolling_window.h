#ifndef ROLLING_WINDOW_H
#define ROLLING_WINDOW_H

typedef struct RollingWindow {
    double *backing_array;
    size_t capacity;
    size_t size;
    size_t front;
} RollingWindow;

void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity);
int mod_rolling_window(int index, int modulo);
void add_data_point(RollingWindow *rw, double dataPoint);

#endif /* ROLLING_WINDOW_H */
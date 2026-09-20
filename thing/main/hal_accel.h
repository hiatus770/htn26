#pragma once
#include <stdbool.h>

void hal_accel_init(void);

// Fills *_mg with milligravity readings if a fresh sample was available
// this call, and returns true. Returns false (leaving outputs untouched)
// when the sensor never initialized or has no new sample yet -- callers
// should hold onto the last good reading rather than treat false as zero.
bool hal_accel_read(float *x_mg, float *y_mg, float *z_mg);

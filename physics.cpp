#include "physics.h"
#include <math.h>

const double g = 9.81; // Gravity constant

double calculateFreeFallTime(double height) {
    return sqrt((2 * height) / g);
}

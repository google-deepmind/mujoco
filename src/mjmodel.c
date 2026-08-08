/* Compute inertia tensor diagonal for a box-shaped fluid */
void mjd_inertiaBoxFluid(double I[3], double mass, double x, double y, double z) {
    I[0] = mass * (y*y + z*z) / 12.0;  /* Ixx */
    I[1] = mass * (x*x + z*z) / 12.0;  /* Iyy */
    I[2] = mass * (x*x + y*y) / 12.0;  /* Izz */
}
#include <stdio.h>
#include "mjmodel.h"

int main() {
    double I[3];
    mjd_inertiaBoxFluid(I, 2.0, 1.0, 2.0, 3.0);
    printf("Ixx=%.4f Iyy=%.4f Izz=%.4f\n", I[0], I[1], I[2]);
    return 0;
}

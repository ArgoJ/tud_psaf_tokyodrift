#ifndef TOKYODRIFT_BEZIER_CURVE_CONFIG_H
#define TOKYODRIFT_BEZIER_CURVE_CONFIG_H

#include <stdint.h>

typedef struct {
    double ts_bezier;
    double bezier_distance;
    double control_point_frac;
} BezierCurveParams;

#endif //TOKYODRIFT_BEZIER_CURVE_CONFIG_H
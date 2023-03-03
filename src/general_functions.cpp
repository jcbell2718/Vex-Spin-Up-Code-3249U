#include "main.h"

okapi::QAngle rotational_distance(okapi::QAngle target, okapi::QAngle current) {
    // Calculates the shortest (directional) distance between two angles in degrees
    double T;
    double C;
    double absolute_minimum;
    T = fmod(target.convert(okapi::degree) + 9000.*360., 360.);
    C = fmod(current.convert(okapi::degree) + 9000.*360., 360.);
    absolute_minimum = std::min({abs(T - C), abs(T - C + 360.), abs(T - C - 360.)});
    if(absolute_minimum == abs(T - C)) return (T - C)*okapi::degree;
    else if(absolute_minimum == abs(T - C + 360)) return (T - C + 360.)*okapi::degree;
    else return (T - C - 360.)*okapi::degree;
}

okapi::QAngle unnormalized_rotation_to(okapi::QAngle target, okapi::QAngle current) {
    // Calculates the un-normalized angle the shortest distance away from the normalized target in degrees
    // i.e. 360 to 20 = 380 rather than 20
    return current + rotational_distance(target, current);
}
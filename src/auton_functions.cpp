#include "main.h"

void drive_to(double target_x, double target_y, double target_theta) {
    // Drives to position on field using PD control
    // Inputs treat home goal as bottom left of first quadrant of coordinate plane
    // with theta counterclockwise from the x-axis
    // PD parameters (require tuning):
    double pkP = .01;
    double pkD = 0;
    double akP = .01;
    double akD = 0;
    int delay;
    // Target theta is counterclockwise, so I negate it here to convert to clockwise
    double a_error = rotational_distance(-target_theta, chassis_controller -> getState().theta.convert(okapi::degree));
    double a_error_previous;
    double a_error_derivative;
    double x_error = target_x - chassis_controller -> getState().x.convert(okapi::inch);
    double x_error_previous;
    double x_error_derivative;
    // Target y negated because odometry views positive y as opposite direction as I do
    double y_error = -target_y - chassis_controller -> getState().y.convert(okapi::inch);
    double y_error_previous;
    double y_error_derivative;
    double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    while(total_error > 1) {
        a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        // Target theta is counterclockwise, so I negate it here to convert to clockwise
        a_error = rotational_distance(-target_theta, chassis_controller -> getState().theta.convert(okapi::degree));
        x_error = target_x - chassis_controller -> getState().x.convert(okapi::inch);
        // Target y negated because odometry views positive y as opposite direction as I do
        y_error = -target_y - chassis_controller -> getState().y.convert(okapi::inch);
        total_error = abs(a_error) + abs(x_error) + abs(y_error);
        a_error_derivative = (a_error - a_error_previous)/(delay/1000.);
        x_error_derivative = (x_error - x_error_previous)/(delay/1000.);
        y_error_derivative = (y_error - y_error_previous)/(delay/1000.);
        chassis_model -> fieldOrientedXArcade(pkP*x_error + pkD*x_error_derivative, pkP*y_error + pkD*y_error_derivative, akP*a_error + akD*a_error_derivative, chassis_controller -> getState().theta);
        pros::delay(delay);
    }
}
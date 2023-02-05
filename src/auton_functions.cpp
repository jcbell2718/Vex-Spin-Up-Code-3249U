#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

void drive_to(okapi::QLength x, okapi::QLength y, okapi::QAngle theta) {
    // Drives to position on field using PD control
    // Inputs treat home goal corner as bottom left of first quadrant of coordinate plane
    // Input rotations are counterclockwise from x-axis
    // Convert from QLengths to doubles
    double target_x = x.convert(okapi::inch);
    double target_y = -y.convert(okapi::inch); // Convert to inverted okapi y-direction
    double target_theta = -theta.convert(okapi::degree); // Convert to okapi clockwise angles
    // PD parameters (require tuning):
    double pkP = .1;
    double pkD = 0;
    double akP = 0;
    double akD = 0;
    int delay = 20;
    int stability_counter = 0;
    double a_error = rotational_distance(target_theta, chassis_controller -> getState().theta.convert(okapi::degree));
    double x_error = target_x - chassis_controller -> getState().x.convert(okapi::inch);
    double y_error = target_y - chassis_controller -> getState().y.convert(okapi::inch);
    double a_error_previous;
    double x_error_previous;
    double y_error_previous;
    double a_error_derivative;
    double x_error_derivative;
    double y_error_derivative;
    double unadjusted_x_input;
    double unadjusted_y_input;
    double x_input;
    double y_input;
    double input_scaler;
    double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    while(stability_counter < 15) {
        a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        a_error = rotational_distance(target_theta, chassis_controller -> getState().theta.convert(okapi::degree));
        x_error = target_x - chassis_controller -> getState().x.convert(okapi::inch);
        y_error = target_y - chassis_controller -> getState().y.convert(okapi::inch);
        total_error = abs(a_error) + abs(x_error) + abs(y_error);
        a_error_derivative = (a_error - a_error_previous)/(delay/1000.);
        x_error_derivative = (x_error - x_error_previous)/(delay/1000.);
        y_error_derivative = (y_error - y_error_previous)/(delay/1000.);
        if(total_error < 5) stability_counter++; // Counter to determine if the robot has settled
        else stability_counter = 0;
        // Adjustment to make path straight when not at 45 degrees at higher than max speed in one direction
        unadjusted_x_input = pkP*x_error + pkD*x_error_derivative;
        unadjusted_y_input = pkP*y_error + pkD*y_error_derivative;
        // if(unadjusted_x_input > 1 || unadjusted_y_input > 1) input_scaler = std::max(unadjusted_x_input, unadjusted_y_input);
        // else input_scaler = 1;
        // x_input = unadjusted_x_input/input_scaler;
        // y_input = unadjusted_y_input/input_scaler;
        chassis_model -> fieldOrientedXArcade(unadjusted_x_input, unadjusted_y_input, akP*a_error + akD*a_error_derivative, chassis_controller -> getState().theta);
        pros::delay(delay);
    }
}
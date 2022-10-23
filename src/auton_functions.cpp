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
    double pkP = .01;
    double pkD = 0;
    double akP = .01;
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
    double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    std::cout << "[";
    while(stability_counter < 10) {
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
        std::cout << total_error << " "; // Output for PD tuning (formatted for graph in matlab)
        if(total_error < 1) stability_counter++; // Counter to determine if the robot has settled
        else stability_counter = 0;
        chassis_model -> fieldOrientedXArcade(pkP*x_error + pkD*x_error_derivative, pkP*y_error + pkD*y_error_derivative, akP*a_error + akD*a_error_derivative, chassis_controller -> getState().theta);
        pros::delay(delay);
    }
    std::cout << "]" << std::endl;
}
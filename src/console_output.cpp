#include "main.h"
#include "global_variables.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

void console_output_fn() {
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
    while(true) {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "X Position: " << chassis.x_pos.convert(okapi::foot) << std::endl;
        std::cout << "Y Position: " << chassis.y_pos.convert(okapi::foot) << std::endl;
        std::cout << "Robot Orientation: " << chassis.angle.convert(okapi::degree) << std::endl;
        std::cout << "Flywheel RPM: " << turret.launch_RPM << std::endl;
        std::cout << "Turntable Target Angle: " << turret.launch_angle.convert(okapi::degree) << std::endl;
        std::cout << "Relative Launch Angle: " << fmod(turret.launch_angle.convert(okapi::degree) + turret.launch_angle.convert(okapi::degree), 360.) << std::endl;
        pros::delay(1000);
    }
}
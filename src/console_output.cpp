#include "main.h"

void console_output_fn() {
    while(true) {
        // std::cout << "---------------------------------------" << std::endl;
        // std::cout << "X Position: " << chassis.x_pos.convert(okapi::foot) << " ft" << std::endl;
        // std::cout << "Y Position: " << chassis.y_pos.convert(okapi::foot) << " ft" << std::endl;
        // std::cout << "Robot Orientation: " << chassis.angle.convert(okapi::degree) << std::endl;
        // std::cout << "Rotational Distance: " << rotational_distance(0_deg, chassis.controller -> getState().theta).convert(okapi::degree) << std::endl;
        std::cout << "Flywheel Target RPM: " << turret.launch_RPM/6. << std::endl;
        //std::cout << "Flywheel Controller Target RPM: " << turret.flywheel_controller -> getTarget() << std::endl;
        // std::cout << "Flywheel Controller Process Value: " << turret.flywheel_controller -> getProcessValue() << std::endl;
        // std::cout << "Turntable Target Angle: " << turret.launch_angle.convert(okapi::degree) << std::endl;
        // std::cout << "Relative Launch Angle: " << fmod(turret.launch_angle.convert(okapi::degree) + turret.launch_angle.convert(okapi::degree), 360.) << std::endl;
        // std::cout << "Front Left Motor Integrated Encoder:  " << chassis.front_left_mtr.getPosition() << std::endl;
        // std::cout << "Front Right Motor Integrated Encoder: " << chassis.front_right_mtr.getPosition() << std::endl;
        // std::cout << "Back Left Motor Integrated Encoder:   " << chassis.back_left_mtr.getPosition() << std::endl;
        // std::cout << "Back Right Motor Integrated Encoder:  " << chassis.back_right_mtr.getPosition() << std::endl;
        // std::cout << "Hue:  " << intake.roller_optical.get_hue() << std::endl;
        pros::delay(100);
    }
}
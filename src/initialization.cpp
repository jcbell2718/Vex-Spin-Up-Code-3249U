#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    std::cout << "a" << std::endl;
    chassis.build_models();
    turret.build_models();
	
	// // Grafana
    std::cout << "b" << std::endl;
	//set_up_grafana();

	// // Initialize tasks
	pros::Task auto_aiming(auto_aim);
	pros::Task intake_control(intake_regulation_fn);
    pros::Task velocity_recording(velocity_recording_fn);
    pros::Task console_output(console_output_fn);
    pros::Task controller_lcd(controller_lcd_fn);
    std::cout << "c" << std::endl;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // control_phase = "disabled";
}
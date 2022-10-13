#include "main.h"
#include "okapi/api/chassis/controller/chassisController.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Initializes tasks
	pros::Task velocity_recording(velocity_recording_fn);
	pros::Task auto_aiming(auto_aim);
	pros::Task intake_control(intake_regulation_function);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	// Auton selection menu through the controller
	bool auton_selected = false;
	master.setText(0, 0, "Selecting Auton:");
	master.setText(1, 0, auton.c_str());
	while(auton_selected == false) {
		if(master_X.changedToPressed()) {
			// Submit choice
			auton_selected = true;
			pros::delay(120); // Clear function is blocked for 110 ms, ensures detection
		} else if(master_left.changedToPressed()) {
			// Cycle through choices backwards
			auton_index--;
			if(auton_index < 0) auton_index = sizeof(auton_list) - 1;
			auton = auton_list[auton_index];
			pros::delay(120); // Clear function is blocked for 110 ms, ensures detection
			master.clearLine(1);
			master.setText(1, 0, auton.c_str());
		} else if(master_right.changedToPressed()) {
			// Cycle through choices forwards
			auton_index++;
			if(auton_index == sizeof(auton_list)) auton_index = 0;
			auton = auton_list[auton_index];
			pros::delay(120); // Clear function is blocked for 110 ms, ensures detection
			master.clearLine(1);
			master.setText(1, 0, auton.c_str());
		}
		pros::delay(20);
	}
	master.clear();
	master.setText(0, 0, "Selected Auton:");
	master.setText(1, 0, auton.c_str());
	if(auton == "Auton 1") {
		chassis_controller -> setState({1_ft, 9_ft, 0_deg});
		turret_controller -> tarePosition();
		inertial.tare();
		pros::delay(5000);
	} else if(auton == "Auton 2") {
		// Paths Here
	}
	master.setText(2, 0, " -Paths Loaded-");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if(auton == "Auton 1") {
		auto_aim_enabled = true;
		drive_to(1.5_ft, 9.5_ft, 45_deg);
		chassis_model -> setMaxVelocity(chassis_max_vel/3.);
		drive_to(9.5_ft, 1.5_ft, 45_deg);
		chassis_model -> setMaxVelocity(chassis_max_vel);
		drive_to(9_ft, 1_ft, -45_deg);
		chassis_model -> setMaxVelocity(chassis_max_vel/3.);
		drive_to(2_ft, 8_ft, -45_deg);
		chassis_model -> setMaxVelocity(chassis_max_vel);
		drive_to(2_ft, 8_ft, 0_deg);
	} else if(auton == "Auton 2") {
		// Auton Code
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	master.clear();
	okapi::Timer LCD_timer = okapi::Timer();
	double power = 0;
	double turning = 0;
	double strafe = 0;
	while (true) {
		// Controller LCD Displays
		if(LCD_timer.repeat(200_ms)) {
			if(auto_aim_enabled) {
				master.setText(0, 0, "Auto-Aim: ON");
				partner.setText(0, 0, "Auto-Aim: ON");
				if(aiming_for_low_goal) {
					master.setText(1, 0, "Target: Low");
					partner.setText(1, 0, "Target: Low");
			 	} else {
				master.setText(1, 0, "Target: High");
				partner.setText(1, 0, "Target: High");
				}
			} else {
				master.setText(0, 0, "Auto-Aim: OFF");
				partner.setText(0, 0, "Auto-Aim: OFF");
				master.setText(1, 0, "RPM:");
				master.setText(1, 5, std::to_string(flywheel_controller -> getTarget()));
				partner.setText(1, 0, "RPM:");
				partner.setText(1, 5, std::to_string(flywheel_controller -> getTarget()));
			} 
			if(front_left_mtr.isOverTemp() || front_right_mtr.isOverTemp() || back_left_mtr.isOverTemp() || back_right_mtr.isOverTemp() || turret_mtr.isOverTemp() || intake_mtr.isOverTemp() || flywheel_mtr_1.isOverTemp() || flywheel_mtr_2.isOverTemp()) {
				master.setText(2, 0, "Over Temp!");
				partner.setText(2, 0, "Over Temp!");
			} else {
				master.clearLine(2);
				partner.clearLine(2);
			}
		}

		// Chassis Control
		if(abs(master.getAnalog(okapi::ControllerAnalog::leftY)) < 2) power = 0;
		else power = powf(master.getAnalog(okapi::ControllerAnalog::leftY), 3.);
		if(abs(master.getAnalog(okapi::ControllerAnalog::leftX)) < 2) strafe = 0;
		else strafe = powf(master.getAnalog(okapi::ControllerAnalog::leftX), 3.);
		if(abs(master.getAnalog(okapi::ControllerAnalog::rightX)) < 2) turning = 0;
		else turning = powf(master.getAnalog(okapi::ControllerAnalog::rightX), 3.);
		if(master_L1.isPressed()) {
			chassis_model -> setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			chassis_model -> stop();
		} else {
			chassis_model -> setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			chassis_model -> xArcade(strafe, power, turning);
		}

		// Auto-aim toggle
		if(partner_X.changedToPressed()) auto_aim_enabled = true;
		else if(partner_Y.changedToPressed()) {
			auto_aim_enabled = false;
			// Truncates flywheel RPM to be a multiple of 50 (manual increment amount)
			flywheel_controller -> setTarget(static_cast<int>(flywheel_controller -> getTarget()) - static_cast<int>(flywheel_controller -> getTarget()) % 50);
		}

		// Manual aiming
		if(!auto_aim_enabled) {
			// Negative arctan to convert to counterclockwise
			turret_controller -> setTarget(unnormalized_rotation_to(-atan2(partner.getAnalog(okapi::ControllerAnalog::leftY), partner.getAnalog(okapi::ControllerAnalog::leftX))/(2.*PI)*360. - inertial.get_yaw(), (turret_controller -> getTarget())));
			if(partner_R1.changedToPressed()) flywheel_controller -> setTarget((flywheel_controller -> getTarget()) + 50);
			else if(partner_R2.changedToPressed()) flywheel_controller -> setTarget((flywheel_controller -> getTarget()) - 50);
		}

		// Intake PTO
		if(partner_A.changedToPressed()) intake_PTO.set_value(true);
		else if(partner_A.changedToReleased()) intake_PTO.set_value(false);

		pros::delay(20);
	}
}
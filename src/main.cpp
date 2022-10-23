#include "main.h"
#include "global_variables.hpp"
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

	// Auton selection menu through the controller
	bool auton_selected = false;
	master.setText(0, 0, "Selecting Auton:   ");
	pros::delay(120);
	master.setText(1, 0, auton.c_str());
	pros::delay(120);
	while(auton_selected == false) {
		if(master_X.changedToPressed()) {
			// Submit choice
			auton_selected = true;
		} else if(master_left.changedToPressed()) {
			// Cycle through choices backwards
			auton_index--;
			if(auton_index < 0) auton_index = sizeof(auton_list) - 1;
			auton = auton_list[auton_index];
			master.setText(1, 0, auton.c_str());
			pros::delay(120);
		} else if(master_right.changedToPressed()) {
			// Cycle through choices forwards
			auton_index++;
			if(auton_index == sizeof(auton_list)) auton_index = 0;
			auton = auton_list[auton_index];
			master.setText(1, 0, auton.c_str());
			pros::delay(120);
		}
		pros::delay(20);
	}
	master.setText(0, 0, "Selected Auton:    ");
	pros::delay(120);
	if(auton == "PD Tuning") {
		chassis_controller -> setState({0_ft, 0_ft, 0_deg});
		turret_controller -> tarePosition();
		inertial.tare();
		pros::delay(5000);
	} else if(auton == "Odometry Tuning") {
		chassis_controller -> setState({0_ft, 0_ft, 0_deg});
		turret_controller -> tarePosition();
		inertial.tare();
		pros::delay(5000);
	} else if(auton == "Roller Start Double Sweep") {
		chassis_controller -> setState({1_ft, -9_ft, 0_deg});
		turret_controller -> tarePosition();
		inertial.tare();
		pros::delay(5000);
	}
	master.setText(2, 0, " -Setup Loaded-    ");
	pros::delay(120);
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
	pros::Mutex target_mutex;
	if(auton == "PD Tuning") {
		// PD controller tuning
		auto_aim_enabled = false;
		drive_to(0_ft, 0_ft, 90_deg);
	} else if(auton == "Odometry Tuning") {
		// Square test to calibrate odometry
		auto_aim_enabled = false;
		while(true) {
			chassis_controller -> driveToPoint({4_ft, 0_ft});
			chassis_controller -> driveToPoint({4_ft, -4_ft});
			chassis_controller -> driveToPoint({0_ft, -4_ft});
			chassis_controller -> driveToPoint({0_ft, 0_ft});
		}
	} else if(auton == "Roller Start Double Sweep") {
		// Starts at roller side, sweeps along the auton line then back along the line of disks on our side
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
	}
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
void competition_initialize() {}

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
	okapi::Timer LCD_timer;
	LCD_timer.placeMark();
	double power = 0;
	double turning = 0;
	double strafe = 0;
	int x_reset_angle_tilt;
	int y_reset_angle_tilt;
	pros::Mutex target_mutex;
	pros::Mutex reset_mutex;
	while (true) {
		// Controller LCD Displays
		if(LCD_timer.getDtFromMark().convert(okapi::millisecond) > 333) {
			if(auto_aim_enabled) {
				master.setText(0, 0, "Auto-Aim: ON       ");
				partner.setText(0, 0, "Auto-Aim: ON       ");
			} else {
				master.setText(0, 0, "Auto-Aim: OFF      ");
				partner.setText(0, 0, "Auto-Aim: OFF      ");
			}
			LCD_timer.placeMark();
		} else if(LCD_timer.getDtFromMark().convert(okapi::millisecond) > 222) {
			if(auto_aim_enabled) {
				if(aiming_for_low_goal) {
					master.setText(1, 0, "Target: Low        ");
					partner.setText(1, 0, "Target: Low        ");
			 	} else {
					master.setText(1, 0, "Target: High       ");
					partner.setText(1, 0, "Target: High       ");
				}
			} else {
				master.setText(1, 0, "RPM:");
				master.setText(1, 5, std::to_string(flywheel_controller -> getTarget()));
				partner.setText(1, 0, "RPM:");
				partner.setText(1, 5, std::to_string(flywheel_controller -> getTarget()));
			}
		} else if(LCD_timer.getDtFromMark().convert(okapi::millisecond) > 111) {
			if(front_left_mtr.isOverTemp() || front_right_mtr.isOverTemp() || back_left_mtr.isOverTemp() || back_right_mtr.isOverTemp() || turret_mtr.isOverTemp() || intake_mtr.isOverTemp() || flywheel_mtr_1.isOverTemp() || flywheel_mtr_2.isOverTemp()) {
				master.setText(2, 0, "Over Temp!         ");
				partner.setText(2, 0, "Over Temp!         ");
			} else {
				master.clearLine(2);
				partner.clearLine(2);
			}
		}

		// Chassis Control
		if(abs(master.getAnalog(okapi::ControllerAnalog::leftY)) < .02) power = 0;
		else power = powf(master.getAnalog(okapi::ControllerAnalog::leftY), 3.);
		if(abs(master.getAnalog(okapi::ControllerAnalog::leftX)) < .02) strafe = 0;
		else strafe = powf(master.getAnalog(okapi::ControllerAnalog::leftX), 3.);
		if(abs(master.getAnalog(okapi::ControllerAnalog::rightX)) < .02) turning = 0;
		else turning = powf(master.getAnalog(okapi::ControllerAnalog::rightX), 3.);
		if(master_L1.isPressed()) {
			chassis_model -> setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			chassis_model -> stop();
		} else {
			chassis_model -> setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			chassis_model -> xArcade(strafe, power, turning);
		}

		// Auto-Aim Toggle
		if(partner_X.changedToPressed()) auto_aim_enabled = true;
		else if(partner_Y.changedToPressed()) {
			auto_aim_enabled = false;
			// Truncates flywheel RPM to be a multiple of 50 (manual increment amount)
			flywheel_controller -> setTarget(static_cast<int>(flywheel_controller -> getTarget()) - static_cast<int>(flywheel_controller -> getTarget()) % 50);
		}

		// Manual Aiming
		if(!auto_aim_enabled) {
			// Negative arctan to convert to counterclockwise
			turret_controller -> setTarget(unnormalized_rotation_to(-atan2(partner.getAnalog(okapi::ControllerAnalog::leftY), partner.getAnalog(okapi::ControllerAnalog::leftX))/(2.*PI)*360. - inertial.get_yaw(), (turret_controller -> getTarget())));
			if(partner_R1.changedToPressed()) flywheel_controller -> setTarget((flywheel_controller -> getTarget()) + 50);
			else if(partner_R2.changedToPressed()) flywheel_controller -> setTarget((flywheel_controller -> getTarget()) - 50);
		}

		// Intake PTO
		if(partner_A.changedToPressed() && !partner_B.isPressed()) intake_PTO.set_value(true);
		else if(partner_A.changedToReleased() && !partner_B.isPressed()) intake_PTO.set_value(false);

		// Odometry Reset Control
		if(partner_B.changedToReleased() && !partner_A.isPressed()) {
			reset_mutex.take();
			if(partner_left.isPressed()) {
				chassis_controller -> setState({chassis_controller -> getState().x, -(12*12-9)*okapi::inch, chassis_controller -> getState().theta});
			} else if(partner_right.isPressed()) {
				chassis_controller -> setState({chassis_controller -> getState().x, -9*okapi::inch, chassis_controller -> getState().theta});
			}
			if(partner_up.isPressed()) {
				chassis_controller -> setState({(12*12-9)*okapi::inch, chassis_controller -> getState().y, chassis_controller -> getState().theta});
			} else if(partner_down.isPressed()) {
				chassis_controller -> setState({9*okapi::inch, chassis_controller -> getState().y, chassis_controller -> getState().theta});
			}
			reset_mutex.give();
		} else if(partner_B.changedToReleased()) { 
			reset_mutex.take();
			if(partner_left.isPressed()) {
				y_reset_angle_tilt = 1;
			} else if(partner_right.isPressed()) {
				y_reset_angle_tilt = -1;
			} else {
				y_reset_angle_tilt = 0;
			}
			if(partner_up.isPressed()) {
				x_reset_angle_tilt = 1;
			} else if(partner_down.isPressed()) {
				x_reset_angle_tilt = -1;
			} else {
				x_reset_angle_tilt = 0;
			}
			chassis_controller -> setState({chassis_controller -> getState().x, chassis_controller -> getState().y, -atan2(x_reset_angle_tilt, y_reset_angle_tilt)*okapi::degree});
			reset_mutex.give();
		}

		pros::delay(20);
	}
}
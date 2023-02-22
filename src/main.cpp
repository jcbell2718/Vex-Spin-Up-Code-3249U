#include "main.h"

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
    // while(control_phase != "opcontrol" && !pros::competition::is_connected()) pros::delay(20);

    // Chassis chassis = *chassis_pointer;
    // Intake intake = *intake_pointer;
    // Turret turret = *turret_pointer;
    // control_phase = "opcontrol";
	// okapi::Timer expansion_timer;
	// expansion_timer.placeMark();
	// double power;
	// double turning;
	// double strafe;

	// turret.auto_aim_enabled = false;
	// turret.set_target_RPM(3400.);
	// intake.intake_mode = true;
	// while(true) {
	// 	// Chassis Control
    //     power = powf(master.getAnalog(okapi::ControllerAnalog::leftY), 3.);
	// 	strafe = powf(master.getAnalog(okapi::ControllerAnalog::leftX), 3.);
	// 	turning = powf(master.getAnalog(okapi::ControllerAnalog::rightX), 3.);
	// 	if(master_L1.isPressed()) {
	// 		chassis.model -> setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// 		chassis.model -> stop();
	// 	} else if(master_R1.isPressed() || master_R2.isPressed()) {
	// 		chassis.model -> setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    //         chassis.drive_raw(.65*strafe, .65*power, .65*turning);
	// 	} else {
	// 		chassis.model -> setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// 		chassis.drive_raw(strafe, power, turning);
	// 	}

	// 	// Auto-Aim Toggle
	// 	if(partner_X.changedToPressed()) turret.auto_aim_enabled = true;
	// 	else if(partner_Y.changedToPressed()) {
	// 		turret.auto_aim_enabled = false;
	// 		// Truncates flywheel RPM to be a multiple of 50 (manual increment amount)
	// 		turret.set_target_RPM(static_cast<int>(turret.launch_RPM) - (static_cast<int>(turret.launch_RPM) % 50));
	// 	}

	// 	// Manual Aiming
	// 	if(!turret.auto_aim_enabled) {
	// 		if(partner_L1.changedToPressed()) turret.set_target_RPM(turret.launch_RPM + 100);
	// 		else if(partner_L2.changedToPressed()) turret.set_target_RPM(turret.launch_RPM - 100);
	// 		if(partner_R1.changedToPressed()) turret.set_target_RPM(turret.launch_RPM + 50);
	// 		else if(partner_R2.changedToPressed()) turret.set_target_RPM(turret.launch_RPM - 50);
	// 	}

	// 	// Intake PTO
	// 	if(partner_left.changedToPressed()) intake.PTO_to_roller_mech();
	// 	else if(partner_right.changedToReleased()) intake.PTO_to_intake();

	// 	// Expansion Release
	// 	if(expansion_timer.millis() >= 95_s &&  partner_down.changedToPressed()) expansion.set_value(true);
	// 	else if(partner_down.changedToPressed()) expansion.set_value(false);

	// 	// Odometry Reset Control
	// 	if(master_B.changedToReleased() && partner_A.isPressed()) {
	// 		chassis.align_odometry_orientation(master_up.isPressed() - master_down.isPressed(), master_left.isPressed() - master_right.isPressed());
	// 	} else if(master_B.changedToReleased()) {
    //         chassis.align_odometry_position(master_up.isPressed() - master_down.isPressed(), master_left.isPressed() - master_right.isPressed());
    //     }

	// 	pros::delay(20);
	// }
}
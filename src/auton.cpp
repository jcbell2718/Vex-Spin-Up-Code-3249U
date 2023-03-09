#include "main.h"

int auton_index = 0;
std::vector<std::string> auton_list = {"None", "PD Tuning", "Roller Start Double Sweep", "Shimmy-Shake", "Roller Only", "Skills 1"};
std::string auton = auton_list[auton_index];

void auton_setup() {
    chassis.reset_encoders();
    turret.turret_controller -> tarePosition();
    inertial.tare();
    if(auton == "Roller Start Double Sweep") chassis.set_position(1_ft, 9_ft, 0_deg);
    else if(auton == "Roller Only") chassis.set_position(1_ft, 9_ft, 0_deg);
	else if(auton == "Skills 1") {
		chassis.set_position(5_ft, 11_ft, 270_deg);
		turret.set_target_angle(-80_deg);
		intake.PTO_to_roller_mech();
	} else chassis.set_position(0_ft, 0_ft, 0_deg);
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
    control_phase = "autonomous";
	if(auton == "PD Tuning") {
		// Square test to calibrate odometry / PD drive
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		while(true) {
			chassis.drive_to_PD(2_ft, 0_ft, 0_deg);
			chassis.drive_to_PD(2_ft, 2_ft, 0_deg);
			chassis.drive_to_PD(0_ft, 2_ft, 0_deg);
			chassis.drive_to_PD(0_ft, 0_ft, 0_deg);
		}
	} else if(auton == "Roller Start Double Sweep") {
		// Starts at roller side, sweeps along the auton line then back along the line of disks on our side
		turret.auto_aim_enabled = true;
		intake.intake_mode = true;
		chassis.drive_to_PD(1.5_ft, 9.5_ft, 45_deg);
		chassis.drive_to_PD(9_ft, 2_ft, 45_deg);
		chassis.drive_to_PD(8_ft, 2_ft, 45_deg);
		chassis.drive_to_PD(8_ft, 2_ft, 135_deg);
		chassis.drive_to_PD(2_ft, 8_ft, 135_deg);
		chassis.drive_to_PD(2_ft, 8_ft, 0_deg);
	} else if(auton == "Shimmy-Shake") {
		turret.auto_aim_enabled = false;
		intake.intake_mode = true;
		chassis.drive_raw(0, 1, 0);
		pros::delay(3000);
		chassis.drive_raw(0, -1, 0);
		pros::delay(1500);
		chassis.drive_raw(0, 0, 0);
	} else if(auton == "Roller Only") {
		// Drive backwards and turn the roller
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		intake.PTO.set_value(true);
		chassis.drive_raw(0, -1, 0);
		pros::delay(200);
		chassis.drive_raw(0, 0, 0);
		intake.turn_roller();
		chassis.drive_raw(0, 1, 0);
		pros::delay(200);
		chassis.drive_raw(0, 0, 0);
		pros::delay(5000);
		intake.PTO.set_value(false);
	} else if(auton == "Skills 1") {
		// 2 rollers, match loads, other 2 rollers, more match loads, expansion
		turret.auto_aim_enabled = false;
		turret.set_target_RPM(2600);
		// Roller 1
		chassis.drive_to_PD(2.5_ft, 11_ft, 270_deg);
		chassis.drive_to_PD(2.5_ft, 11.5_ft, 270_deg, 2_s);
		intake.turn_roller();
		std::cout << "Roller 1" << std::endl;
		// Roller 2
		chassis.drive_to_PD(2_ft, 9.5_ft, 0_deg);
		chassis.drive_to_PD(.75_ft, 9.5_ft, 0_deg, 2_s);
		intake.turn_roller();
		intake.PTO_to_intake();
		std::cout << "Roller 2" << std::endl;
		// Match loads
		chassis.drive_to_PD(2_ft, 6_ft, 180_deg);
		intake.index();
		for(int i = 0; i < 8; i++) {
			chassis.drive_to_PD(1_ft, 6_ft, 180_deg);
			chassis.drive_to_PD(2_ft, 6_ft, 180_deg);
			intake.index();
		}
		std::cout << "Match Loads" << std::endl;
		// Roller 3
		intake.PTO_to_roller_mech();
		chassis.drive_to_PD(1_ft, 6_ft, 90_deg);
		chassis.drive_to_PD(1_ft, 1_ft, 90_deg);
		chassis.drive_to_PD(9.5_ft, 1_ft, 90_deg);
		chassis.drive_to_PD(9.5_ft, .5_ft, 90_deg, 2_s);
		intake.turn_roller();
		std::cout << "Roller 3" << std::endl;
		// Roller 4
		chassis.drive_to_PD(10_ft, 2.5_ft, 0_deg);
		chassis.drive_to_PD(12_ft, 2.5_ft, 0_deg, 2_s);
		intake.turn_roller();
		std::cout << "Roller 4" << std::endl;
		// Match loads
		chassis.drive_to_PD(10_ft, 6_ft, 0_deg);
		for(int i = 0; i < 7; i++) {
			chassis.drive_to_PD(11_ft, 6_ft, 0_deg);
			chassis.drive_to_PD(10_ft, 6_ft, 0_deg);
			intake.index();
		}
		std::cout << "Match Loads" << std::endl;
		// Expansion
		chassis.drive_to_PD(6_ft, 6_ft, -45_deg);
		expansion.set_value(true);
		std::cout << "Expansion" << std::endl;
	}
}
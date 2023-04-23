#include "main.h"

int auton_index = 0;
std::vector<std::string> auton_list = {"Testing", "None", "Left WP", "Right WP", "Roller Only", "Double Sweep", "Skills 1", "PD Tuning", "Spin Test"};
std::string auton = auton_list[auton_index];

void auton_setup() {
    chassis.reset_encoders();
    turret.turret_controller -> tarePosition();
    inertial.tare();
	if(auton == "Testing") {
		is_skills = true;
		chassis.set_position(10.5_in, 110_in, 0_deg);
	} else if(auton == "Double Sweep") {
		is_skills = false;
		chassis.set_position(10.5_in, 110_in, 0_deg);
	} else if(auton == "Roller Only") {
		is_skills = false;
		chassis.set_position(10.5_in, 110_in, 0_deg);
	} else if(auton == "Left WP") {
		is_skills = false;
		chassis.set_position(10.5_in, 110_in, 0_deg);
		intake.PTO_to_roller_mech();
	} else if(auton == "Right WP") {
		is_skills = false;
		chassis.set_position(78.5_in, 10.5_in, 90_deg);
		intake.PTO_to_roller_mech();
	} else if(auton == "Skills 1") {
		is_skills = true;
		chassis.set_position(23.5_in/2* 5, 23.5_in/2* 11, 270_deg);
		turret.set_target_angle(-100_deg);
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
	if(auton == "Testing") {
		turret.auto_aim_enabled = true;
		intake.PTO_to_intake();
		while(true) {
			pros::delay(100);
		}
	} else if(auton == "PD Tuning") {
		// Square test to calibrate odometry / PD drive
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		while(true) {
			chassis.drive_to_PD(2_ft, 0_ft, 0_deg);
			chassis.drive_to_PD(2_ft, 2_ft, 0_deg);
			chassis.drive_to_PD(0_ft, 2_ft, 0_deg);
			chassis.drive_to_PD(0_ft, 0_ft, 0_deg);
		}
	} else if(auton == "Spin Test") {
		// Square test to calibrate odometry / PD drive
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		while(true) {
			chassis.drive_to_PD(0_ft, 0_ft, 90_deg);
			pros::delay(500);
			chassis.drive_to_PD(0_ft, 0_ft, 180_deg);
			pros::delay(500);
			chassis.drive_to_PD(0_ft, 0_ft, 270_deg);
			pros::delay(500);
			chassis.drive_to_PD(0_ft, 0_ft, 0_deg);
			pros::delay(500);
		}
	} else if(auton == "Double Sweep") {
		// Starts at roller side, sweeps along the auton line then back along the line of disks on our side
		turret.auto_aim_enabled = true;
		intake.intake_mode = true;
		chassis.drive_to_PD(1.5_ft, 9.5_ft, 45_deg);
		chassis.drive_to_PD(9_ft, 2_ft, 45_deg);
		chassis.drive_to_PD(8_ft, 2_ft, 45_deg);
		chassis.drive_to_PD(8_ft, 2_ft, 135_deg);
		chassis.drive_to_PD(2_ft, 8_ft, 135_deg);
		chassis.drive_to_PD(2_ft, 8_ft, 0_deg);
	} else if(auton == "Roller Only") {
		// Drive backwards and turn the roller
		turret.auto_aim_enabled = false;
		intake.PTO_to_roller_mech();
		chassis.drive_to_PD(1._ft, 9.5_ft, 0_deg);
		chassis.drive_to_PD(.65_ft, 9.5_ft, 0_deg, .5_s);
		intake.turn_roller();
		chassis.drive_to_PD(1._ft, 9.5_ft, 0_deg);
		intake.PTO_to_intake();
	} else if(auton == "Left WP") {
		// Shoots 2 preloads, turns the roller, shoots near triple disk stack
		turret.auto_aim_enabled = true;
		// Roller
		chassis.drive_to_PD(8_in, 110_in, 0_deg, 1_s, .5_s);
		intake.turn_roller();
		chassis.drive_to_PD(11_in, 110_in, 0_deg);
		pros::delay(1000);
		// Preloads
		intake.PTO_to_intake();
		while(intake.disk_switch.get_value()) pros::delay(20);
		pros::delay(2500);
		// Triple stack
		// Disk 1
		okapi::Timer override_timer;
		chassis.drive_to_PD(23._in*(1.+.5), 23._in*(4.-.5), -45_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
		// Disk 2
		chassis.drive_to_PD(23._in*(1.+.75), 23._in*(4.-.57), -45_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
		// Disk 3
		chassis.drive_to_PD(23._in*(1.+1.), 23._in*(4.-1.), -45_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
	} else if(auton == "Right WP") {
		// Shoots 2 preloads, turns the roller, shoots near triple disk stack
		turret.auto_aim_enabled = true;
		// Roller
		chassis.drive_to_PD(110_in, 15_in, 90_deg, 3_s);
		chassis.drive_to_PD(110_in, 8_in, 90_deg, 3_s);
		intake.turn_roller();
		chassis.drive_to_PD(110_in, 15_in, 90_deg);
		// Preloads
		intake.PTO_to_intake();
		while(intake.disk_switch.get_value()) pros::delay(20);
		pros::delay(2500);
		// Triple stack
		// Disk 1
		okapi::Timer override_timer;
		chassis.drive_to_PD(23._in*(4.+.25), 23._in*(1.-.25), 135_deg);
		chassis.drive_to_PD(23._in*(4.-.75), 23._in*(1.+.75), 135_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
		// Disk 2
		chassis.drive_to_PD(23._in*(4.-1.), 23._in*(1.+1.), 135_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
		// Disk 3
		chassis.drive_to_PD(23._in*(4.-1.25), 23._in*(1.+1.25), 135_deg);
		override_timer.placeMark();
		while(!intake.indexing && override_timer.getDtFromMark() < 3_s) pros::delay(20);
		pros::delay(1000);
	} else if(auton == "Skills 1") {
		// 2 rollers, match loads, other 2 rollers, more match loads, expansion
		turret.auto_aim_enabled = false;
		turret.set_target_RPM(2600);
		// Roller 1
		chassis.drive_to_PD(23.5_in/2* 2.5, 23.5_in/2* 11, 270_deg);
		chassis.drive_to_PD(23.5_in/2* 2.5, 23.5_in/2* 11.25, 270_deg, 1_s);
		intake.turn_roller();
		std::cout << "Roller 1" << std::endl;
		// Roller 2
		chassis.drive_to_PD(23.5_in/2* 2.5, 23.5_in/2* 9.5, 0_deg);
		chassis.drive_to_PD(23.5_in/2* .75, 23.5_in/2* 9.5, 0_deg, 1_s);
		intake.turn_roller();
		intake.PTO_to_intake();
		std::cout << "Roller 2" << std::endl;
		// Match loads
		chassis.drive_to_PD(23.5_in/2* 2, 23.5_in/2* 6, 180_deg);
		intake.index();
		for(int i = 0; i < 8; i++) {
			chassis.drive_to_PD(23.5_in/2* 1, 23.5_in/2* 6, 180_deg, 2_s);
			chassis.drive_to_PD(23.5_in/2* 2, 23.5_in/2* 6, 180_deg, 2_s);
			pros::delay(1000);
			intake.index();
		}
		std::cout << "Match Loads" << std::endl;
		// Roller 3
		intake.PTO_to_roller_mech();
		chassis.drive_to_PD(23.5_in/2* 1, 23.5_in/2* 6, 90_deg, 5_s);
		chassis.drive_to_PD(23.5_in/2* 1, 23.5_in/2* 1, 90_deg, 5_s);
		chassis.drive_to_PD(23.5_in/2* 9.5, 23.5_in/2* 1, 90_deg, 5_s);
		chassis.drive_to_PD(23.5_in/2* 9.5, 23.5_in/2* .5, 90_deg, 2_s);
		intake.turn_roller();
		std::cout << "Roller 3" << std::endl;
		// Roller 4
		chassis.drive_to_PD(23.5_in/2* 10, 23.5_in/2* 2.5, 0_deg);
		chassis.drive_to_PD(23.5_in/2* 12, 23.5_in/2* 2.5, 0_deg, 2_s);
		intake.turn_roller();
		std::cout << "Roller 4" << std::endl;
		// Match loads
		chassis.drive_to_PD(23.5_in/2* 10, 23.5_in/2* 6, 0_deg);
		for(int i = 0; i < 7; i++) {
			chassis.drive_to_PD(23.5_in/2* 11, 23.5_in/2* 6, 0_deg, 2_s);
			chassis.drive_to_PD(23.5_in/2* 10, 23.5_in/2* 6, 0_deg, 2_s);
			pros::delay(1000);
			intake.index();
		}
		std::cout << "Match Loads" << std::endl;
		// Expansion
		chassis.drive_to_PD(23.5_in/2* 6, 23.5_in/2* 6, -45_deg);
		expansion.expand();
		std::cout << "Expansion" << std::endl;
	}
}
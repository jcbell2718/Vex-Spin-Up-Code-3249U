#include "main.h"

int auton_index = 0;
std::vector<std::string> auton_list = {"None", "PD Tuning", "Odometry Tuning", "Roller Start Double Sweep", "Shimmy-Shake", "Roller Only", "Blind Shot"};
std::string auton = auton_list[auton_index];

void auton_setup() {
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
    chassis.reset_encoders();
    turret.turret_controller -> tarePosition();
    inertial.tare();
    pros::delay(120);
    if(auton == "PD Tuning") {
        chassis.controller -> setState({0_ft, 0_ft, 0_deg});
    } else if(auton == "Odometry Tuning") {
        chassis.controller -> setState({0_ft, 0_ft, 0_deg});
    } else if(auton == "Roller Start Double Sweep") {
        chassis.controller -> setState({1_ft, -9_ft, 0_deg});
    } else if(auton == "Roller Only") {
        chassis.controller -> setState({1_ft, -9_ft, 0_deg});
    } else if(auton == "Blind Shot") {
        chassis.controller -> setState({0_ft, 0_ft, 0_deg});
    }
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
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
	pros::Mutex target_mutex;
	if(auton == "PD Tuning") {
		// PD controller tuning
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		chassis.drive_to_PD(2_ft, 0_ft, 0_deg);
	} else if(auton == "Odometry Tuning") {
		// Square test to calibrate odometry
		turret.auto_aim_enabled = false;
		intake.intake_mode = false;
		while(true) {
			chassis.drive_to_default_odom(4_ft, 0_ft);
			chassis.drive_to_default_odom(4_ft, 4_ft);
			chassis.drive_to_default_odom(0_ft, 4_ft);
			chassis.drive_to_default_odom(0_ft, 0_ft);
		}
	} else if(auton == "Roller Start Double Sweep") {
		// Starts at roller side, sweeps along the auton line then back along the line of disks on our side
		turret.auto_aim_enabled = true;
		intake.intake_mode = true;
		chassis.drive_to_PD(1.5_ft, 9.5_ft, 45_deg);
		chassis.drive_to_PD(9.5_ft, 1.5_ft, 45_deg);
		chassis.drive_to_PD(9_ft, 1_ft, 135_deg);
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
	} else if(auton == "Blind Shot") {
		// Drive backwards and turn the roller
		turret.auto_aim_enabled = true;
		intake.intake_mode = true;
		pros::delay(5000);
		intake.index();
	}
}
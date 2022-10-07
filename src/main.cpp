#include "main.h"

// Constants
double const launch_height = 16.00;
double const launch_angle = 30.00;
double const offset = 5;
double const alpha = -20;
double const launch_max_vel = 2500./60.*3.25*PI/2.; // rpm / 60 (sec/min) * flywheel diameter * pi. / 2 This doesn't consider frictional losses atm.
double const goal_height = 30.00;
double const goal_x = 10.75*12;
double const goal_y = 10.75*12;
double const low_goal_x = 0;
double const low_goal_y = 0;
double const g = -32.17*12;

// Global Variables
double global_x_vel;
double global_y_vel;
bool auto_aim_enabled = false;
bool aiming_for_low_goal = false;
bool using_gps = false;
int auton_index = 1;
std::string auton_list[3] = {"None", "Auton 1", "Auton 2"};
std::string auton = auton_list[auton_index];

// Chassis is global so internal values aren't reset between competition phases. Also makes edits much easier
okapi::Motor front_left_mtr(FRONT_LEFT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor front_right_mtr(FRONT_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_left_mtr(BACK_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_right_mtr(BACK_RIGHT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::ADIEncoder center_encoder(CENTER_ENCODER_PORT_TOP, CENTER_ENCODER_PORT_BOTTOM, false);
okapi::ADIEncoder left_encoder(LEFT_ENCODER_PORT_TOP, LEFT_ENCODER_PORT_BOTTOM, false);
okapi::ADIEncoder right_encoder(RIGHT_ENCODER_PORT_TOP, RIGHT_ENCODER_PORT_BOTTOM, false);
std::shared_ptr<okapi::OdomChassisController> chassis_controller = okapi::ChassisControllerBuilder()
	.withMotors(front_left_mtr, front_right_mtr, back_left_mtr, back_right_mtr)
	.withSensors(left_encoder, right_encoder, center_encoder)
	// Blue gearset, no external gear ratio, 3.25 inch diameter wheels, 14.75 inch wheel track
	.withDimensions({okapi::AbstractMotor::gearset::blue, (1./1.)}, {{3.25_in, 14.75_in, 0_in, 3.25_in}, okapi::imev5BlueTPR})
	// Tracking wheels diameter (2.75 in) and track (7 in), middle encoder distance (1 in) and diameter (2.75 in), and TPR
	.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, okapi::quadEncoderTPR})
	.buildOdometry();
std::shared_ptr<okapi::ChassisModel> chassis_model = std::shared_ptr<okapi::ChassisModel>(chassis_controller -> getModel());;
std::shared_ptr<okapi::AsyncMotionProfileController> chassis_profile_controller = okapi::AsyncMotionProfileControllerBuilder()
	.withOutput(chassis_controller)
	// Max speed 3.66 m/s, max acceleration 3 m/s^2, max jerk 1000 m/s^3
	.withLimits({3.66, 3, 1000})
	.withLogger(okapi::Logger::getDefaultLogger())
	.buildMotionProfileController();

// Controller for launcher rotation
okapi::Motor turret_mtr(TURRET_MOTOR_PORT, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller = okapi::AsyncPosControllerBuilder()
	.withMotor(turret_mtr)
	.withSensor(okapi::IntegratedEncoder(turret_mtr))
	.withGearset({okapi::AbstractMotor::gearset::red, (1./1.)})
	.withLogger(okapi::Logger::getDefaultLogger())
	.build();

// Controller for flywheel velocity
okapi::Motor flywheel_mtr(FLYWHEEL_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller = okapi::AsyncVelControllerBuilder()
	.withMotor(flywheel_mtr)
	.withSensor(okapi::IntegratedEncoder(flywheel_mtr))
	.withGearset({okapi::AbstractMotor::gearset::blue, (1./1.)})
	.withLogger(okapi::Logger::getDefaultLogger())
	.build();

pros::Imu inertial = pros::Imu(INERTIAL_PORT);
pros::Gps gps(GPS_PORT, 0, 0, 0);
pros::Controller controller_master(pros::E_CONTROLLER_MASTER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	auto_aim_enabled = false; // Terminates aiming program
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

void velocity_recording_fn() {
	// Records robot velocity globally, although specific values are frozen during computations to avoid strange behavior
	double old_x_pos = chassis_controller -> getState().x.convert(okapi::inch);
	double old_y_pos = chassis_controller -> getState().y.convert(okapi::inch);
	double new_x_pos;
	double new_y_pos;
	double delay = 200;
	while(true) {
		pros::delay(delay);
		old_x_pos = new_x_pos;
		old_y_pos = new_y_pos;
		if(using_gps == true) {
			// Not sure if gps output is in meters or millimeters, I'll have to do testing once we start building
			chassis_controller -> setState({gps.get_status().x*okapi::millimeter,gps.get_status().y*okapi::millimeter,gps.get_status().yaw*okapi::degree});
			new_x_pos = chassis_controller -> getState().x.convert(okapi::inch);
			new_y_pos = chassis_controller -> getState().y.convert(okapi::inch);
		} else {
			new_x_pos = chassis_controller -> getState().x.convert(okapi::inch);
			new_y_pos = chassis_controller -> getState().y.convert(okapi::inch);
		}
		global_x_vel = (new_x_pos-old_x_pos)/(delay/1000.);
		global_y_vel = (new_y_pos-old_y_pos)/(delay/1000.);
	}
}

void competition_initialize() {
	// Initializes the velocity recording task
	pros::Task velocity_recording(velocity_recording_fn);
	// Auton selection menu through the controller
	bool auton_selected = false;
	controller_master.print(0, 0, "Selecting Auton:");
	controller_master.print(1, 0, auton.c_str());
	while(auton_selected == false) {
		if (controller_master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			// Submit choice
			auton_selected = true;
			pros::delay(95); // The clear function is blocked for 110 ms, so this helps avoid problems there
		} else if (controller_master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			// Cycle through choices backwards
			auton_index--;
			if(auton_index < 0) auton_index = sizeof(auton_list) - 1;
			auton = auton_list[auton_index];
			pros::delay(95); // The clear function is blocked for 110 ms, so this helps avoid problems there
			// Don't trigger it multiple times with one press
			while(controller_master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) pros::delay(10);
			// Update screen
			controller_master.clear_line(1);
			controller_master.print(1, 0, auton.c_str());
		} else if(controller_master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			// Cycle through choices forwards
			auton_index++;
			if(auton_index == sizeof(auton_list)) auton_index = 0;
			auton = auton_list[auton_index];
			pros::delay(95); // The clear function is blocked for 110 ms, so this helps avoid problems there
			// Don't trigger it multiple times with one press
			while(controller_master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) pros::delay(20);
			// Update screen
			controller_master.clear_line(1);
			controller_master.print(1, 0, auton.c_str());
		}
		pros::delay(20);
	}
	controller_master.clear();
	controller_master.print(0, 0, "Selected Auton:");
	controller_master.print(1, 0, auton.c_str());
	if(auton == "Auton 1") {
		// Paths Here
	} else if(auton == "Auton 2") {
		// Paths Here
	}
	controller_master.print(2, 0, " -Paths Loaded-");
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
		// Auton Code
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
	controller_master.clear();
	while (true) {
		// Update Controller LCD display after every cycle
		if(auto_aim_enabled) {
			controller_master.print(0, 0, "Auto-Aim: ON");
		} else {
			controller_master.print(0, 0, "Auto-Aim: OFF");
		}
		if(aiming_for_low_goal) {
			controller_master.print(1, 0, "Target: Low");
		} else {
			controller_master.print(1, 0, "Target: High");
		}
		pros::delay(20);
	}
}

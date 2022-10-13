#include "main.h"

// Constants
double const launch_height = 16.00;
double const launch_angle = 30.00;
double const offset = 5;
double const alpha = -20;
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
bool indexing = false;
int auton_index = 1;
std::string auton_list[3] = {"None", "Auton 1", "Auton 2"};
std::string auton = auton_list[auton_index];

// Chassis
okapi::Motor front_left_mtr(FRONT_LEFT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor front_right_mtr(FRONT_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_left_mtr(BACK_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_right_mtr(BACK_RIGHT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::ADIEncoder center_encoder(CENTER_ENCODER_PORT_TOP, CENTER_ENCODER_PORT_BOTTOM, false);
okapi::ADIEncoder left_encoder(LEFT_ENCODER_PORT_TOP, LEFT_ENCODER_PORT_BOTTOM, false);
okapi::ADIEncoder right_encoder(RIGHT_ENCODER_PORT_TOP, RIGHT_ENCODER_PORT_BOTTOM, false);
std::shared_ptr<okapi::OdomChassisController> chassis_controller = okapi::ChassisControllerBuilder()
	.withMotors(front_left_mtr, front_right_mtr, back_right_mtr, back_left_mtr)
	.withSensors(left_encoder, right_encoder, center_encoder)
	// Red gearset, 36/84 gear ratio, 3.25 inch diameter wheels, 16.4 inch wheel track
	.withDimensions({okapi::AbstractMotor::gearset::red, (36./84.)}, {{3.25_in, 16.4_in}, okapi::imev5RedTPR})
	// 2.75 in tracking wheel diameter, 7 inch wheel track, 1 inch middle encoder distance
	.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, okapi::quadEncoderTPR})
	.buildOdometry();
std::shared_ptr<okapi::XDriveModel> chassis_model = std::dynamic_pointer_cast<okapi::XDriveModel>(chassis_controller -> getModel());
std::shared_ptr<okapi::AsyncMotionProfileController> chassis_profile_controller = okapi::AsyncMotionProfileControllerBuilder()
	.withOutput(chassis_controller)
	// Max speed 3.66 m/s, max acceleration 3 m/s^2, max jerk 1000 m/s^3
	.withLimits({3.66, 3, 1000})
	.withLogger(okapi::Logger::getDefaultLogger())
	.buildMotionProfileController();

// Controller for launcher rotation
okapi::Motor turret_mtr(TURRET_MOTOR_PORT, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller = okapi::AsyncPosControllerBuilder()
	.withMotor(turret_mtr)
	.withSensor(okapi::IntegratedEncoder(turret_mtr))
	.withGearset({okapi::AbstractMotor::gearset::red, (148./32.)})
	.withLogger(okapi::Logger::getDefaultLogger())
	.build();

// Controller for flywheel velocity
okapi::Motor flywheel_mtr_1(FLYWHEEL_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor flywheel_mtr_2(FLYWHEEL_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::MotorGroup flywheel_mtrs({flywheel_mtr_1, flywheel_mtr_2});
std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller = okapi::AsyncVelControllerBuilder()
	.withMotor(flywheel_mtrs)
	.withSensor(okapi::IntegratedEncoder(flywheel_mtr_1))
	// Not actually, but equivalent. It's really 1:1 3600 rpm
	.withGearset({okapi::AbstractMotor::gearset::blue, (1./6.)})
	.withLogger(okapi::Logger::getDefaultLogger())
	.build();

// Intake and Indexer
okapi::Motor intake_mtr(INTAKE_MOTOR_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Timer intake_timer = okapi::Timer();
okapi::ADIButton disk_switch(LIMIT_SWITCH_PORT);
pros::ADIDigitalOut indexer(INDEXER_PORT);
pros::ADIDigitalOut intake_PTO(PTO_PORT);

// Rollers
pros::Optical roller_optical(ROLLER_OPTICAL_PORT);

// Other sensors
pros::Imu inertial = pros::Imu(INERTIAL_PORT);
pros::Gps gps(GPS_PORT, 0, 0, 0);

// Controllers
okapi::Controller master(okapi::ControllerId::master);
okapi::Controller partner(okapi::ControllerId::master);
okapi::ControllerButton partner_R1(okapi::ControllerId::partner, okapi::ControllerDigital::R1);
okapi::ControllerButton partner_R2(okapi::ControllerId::partner, okapi::ControllerDigital::R2);
okapi::ControllerButton partner_L1(okapi::ControllerId::partner, okapi::ControllerDigital::L1);
okapi::ControllerButton partner_L2(okapi::ControllerId::partner, okapi::ControllerDigital::L2);
okapi::ControllerButton partner_X(okapi::ControllerId::partner, okapi::ControllerDigital::X);
okapi::ControllerButton partner_Y(okapi::ControllerId::partner, okapi::ControllerDigital::Y);
okapi::ControllerButton partner_A(okapi::ControllerId::partner, okapi::ControllerDigital::A);
okapi::ControllerButton master_R1(okapi::ControllerId::master, okapi::ControllerDigital::R1);
okapi::ControllerButton master_R2(okapi::ControllerId::master, okapi::ControllerDigital::R2);
okapi::ControllerButton master_L1(okapi::ControllerId::master, okapi::ControllerDigital::L1);
okapi::ControllerButton master_L2(okapi::ControllerId::master, okapi::ControllerDigital::L2);
okapi::ControllerButton master_X(okapi::ControllerId::master, okapi::ControllerDigital::X);
okapi::ControllerButton master_Y(okapi::ControllerId::master, okapi::ControllerDigital::Y);
okapi::ControllerButton master_left(okapi::ControllerId::master, okapi::ControllerDigital::left);
okapi::ControllerButton master_right(okapi::ControllerId::master, okapi::ControllerDigital::right);
#include "global_variables.hpp"
#include "main.h"

// Constants
double const launch_height = 16.00;
double const launch_angle = 30.00;
double const offset = 5;
double const alpha = -20;
double const goal_height = 30.00;
double const goal_x = 10.5*12;
double const goal_y = 10.5*12;
double const opponent_goal_x = 1.5*12;
double const opponent_goal_y = 1.5*12;
double const alliance_goal_x = 10.5*12;
double const alliance_goal_y = 10.5*12;
double const g = -32.17*12;

// Global Variables
double global_x_vel;
double global_y_vel;
double global_target_x = 10.5*12;
double global_target_y = 10.5*12;
bool intake_enabled = false;
bool auto_aim_enabled = false;
bool aiming_for_low_goal = false;
bool using_gps = false;
bool indexing = false;
bool auton_indexer_trigger = false;
std::string alliance_color = "blue";
int auton_index = 1;
std::vector<std::string> auton_list = {"None", "PD Tuning", "Odometry Tuning", "Roller Start Double Sweep", "Shimmy-Shake", "Roller Only", "Blind Shot"};
std::string auton = auton_list[auton_index];

// Chassis
okapi::Motor front_left_mtr(FRONT_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor front_right_mtr(FRONT_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_left_mtr(BACK_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor back_right_mtr(BACK_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
okapi::ADIEncoder center_encoder(CENTER_ENCODER_PORTS, false);
okapi::ADIEncoder left_encoder(LEFT_ENCODER_PORTS, false);
okapi::ADIEncoder right_encoder(RIGHT_ENCODER_PORTS, true);
std::shared_ptr<okapi::OdomChassisController> chassis_controller;
std::shared_ptr<okapi::XDriveModel> chassis_model;
std::shared_ptr<okapi::AsyncMotionProfileController> chassis_profile_controller;
double chassis_max_vel;

// Controller for launcher rotation
okapi::Motor turret_mtr(TURRET_MOTOR_PORT, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller;

// Controller for flywheel velocity
okapi::Motor flywheel_mtr_1(FLYWHEEL_MOTOR_PORT_1, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor flywheel_mtr_2(FLYWHEEL_MOTOR_PORT_2, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::MotorGroup flywheel_mtrs({flywheel_mtr_1, flywheel_mtr_2});
std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller;

// Intake and Indexer
okapi::Motor intake_mtr(INTAKE_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
okapi::ADIButton disk_switch(LIMIT_SWITCH_PORT);
pros::ADIDigitalOut indexer(INDEXER_PORT);
pros::ADIDigitalOut intake_PTO(PTO_PORT);

// Rollers
pros::Optical roller_optical(ROLLER_OPTICAL_PORT);

// Expansion
pros::ADIDigitalOut expansion(EXPANSION_PORT);

// Other
pros::Imu inertial = pros::Imu(INERTIAL_PORT);
pros::Gps gps(GPS_PORT, 0, 0, 0);
okapi::Timer cout_timer;

// Controllers
okapi::Controller master(okapi::ControllerId::master);
okapi::Controller partner(okapi::ControllerId::partner);
okapi::ControllerButton partner_R1(okapi::ControllerId::partner, okapi::ControllerDigital::R1);
okapi::ControllerButton partner_R2(okapi::ControllerId::partner, okapi::ControllerDigital::R2);
okapi::ControllerButton partner_L1(okapi::ControllerId::partner, okapi::ControllerDigital::L1);
okapi::ControllerButton partner_L2(okapi::ControllerId::partner, okapi::ControllerDigital::L2);
okapi::ControllerButton partner_X(okapi::ControllerId::partner, okapi::ControllerDigital::X);
okapi::ControllerButton partner_Y(okapi::ControllerId::partner, okapi::ControllerDigital::Y);
okapi::ControllerButton partner_A(okapi::ControllerId::partner, okapi::ControllerDigital::A);
okapi::ControllerButton partner_B(okapi::ControllerId::partner, okapi::ControllerDigital::B);
okapi::ControllerButton partner_left(okapi::ControllerId::partner, okapi::ControllerDigital::left);
okapi::ControllerButton partner_right(okapi::ControllerId::partner, okapi::ControllerDigital::right);
okapi::ControllerButton partner_up(okapi::ControllerId::partner, okapi::ControllerDigital::up);
okapi::ControllerButton partner_down(okapi::ControllerId::partner, okapi::ControllerDigital::down);
okapi::ControllerButton master_R1(okapi::ControllerId::master, okapi::ControllerDigital::R1);
okapi::ControllerButton master_R2(okapi::ControllerId::master, okapi::ControllerDigital::R2);
okapi::ControllerButton master_L1(okapi::ControllerId::master, okapi::ControllerDigital::L1);
okapi::ControllerButton master_L2(okapi::ControllerId::master, okapi::ControllerDigital::L2);
okapi::ControllerButton master_X(okapi::ControllerId::master, okapi::ControllerDigital::X);
okapi::ControllerButton master_Y(okapi::ControllerId::master, okapi::ControllerDigital::Y);
okapi::ControllerButton master_A(okapi::ControllerId::master, okapi::ControllerDigital::A);
okapi::ControllerButton master_B(okapi::ControllerId::master, okapi::ControllerDigital::B);
okapi::ControllerButton master_left(okapi::ControllerId::master, okapi::ControllerDigital::left);
okapi::ControllerButton master_right(okapi::ControllerId::master, okapi::ControllerDigital::right);
okapi::ControllerButton master_up(okapi::ControllerId::master, okapi::ControllerDigital::left);
okapi::ControllerButton master_down(okapi::ControllerId::master, okapi::ControllerDigital::right);
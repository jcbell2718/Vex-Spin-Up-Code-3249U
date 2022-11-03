#pragma once
#include "okapi/api/units/QSpeed.hpp"
#ifndef _GLOBAL_VARIABLES_HPP_
#define  _GLOBAL_VARIABLES_HPP_

#include "main.h"

extern double const launch_height;
extern double const launch_angle;
extern double const offset;
extern double const alpha;
extern double const launch_max_vel;
extern double const goal_height;
extern double const goal_x;
extern double const goal_y;
extern double const opponent_goal_x;
extern double const opponent_goal_y;
extern double const alliance_goal_x;
extern double const alliance_goal_y;
extern double const g;
extern double global_x_vel;
extern double global_y_vel;
extern double global_target_x;
extern double global_target_y;
extern double chassis_max_vel;
extern bool intake_enabled;
extern bool auto_aim_enabled;
extern bool aiming_for_low_goal;
extern bool using_gps;
extern bool indexing;
extern std::string alliance_color;
extern int auton_index;
extern std::vector<std::string> auton_list;
extern std::string auton;
extern okapi::Motor front_left_mtr;
extern okapi::Motor front_right_mtr;
extern okapi::Motor back_left_mtr;
extern okapi::Motor back_right_mtr;
extern okapi::ADIEncoder center_encoder;
extern okapi::ADIEncoder left_encoder;
extern okapi::ADIEncoder right_encoder;
extern std::shared_ptr<okapi::OdomChassisController> chassis_controller;
extern std::shared_ptr<okapi::XDriveModel> chassis_model;
extern std::shared_ptr<okapi::AsyncMotionProfileController> chassis_profile_controller;
extern okapi::Motor turret_mtr;
extern std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller;
extern okapi::Motor flywheel_mtr_1;
extern okapi::Motor flywheel_mtr_2;
extern okapi::MotorGroup flywheel_mtrs;
extern std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller;
extern okapi::Motor intake_mtr;
extern okapi::Timer intake_timer;
extern okapi::ADIButton disk_switch;
extern pros::ADIDigitalOut indexer;
extern pros::ADIDigitalOut intake_PTO;
extern pros::Optical roller_optical;
extern pros::ADIDigitalOut expansion;
extern pros::Imu inertial;
extern pros::Gps gps;
extern okapi::Timer cout_timer;
extern okapi::Controller master;
extern okapi::Controller partner;
extern okapi::ControllerButton partner_R1;
extern okapi::ControllerButton partner_R2;
extern okapi::ControllerButton partner_L1;
extern okapi::ControllerButton partner_L2;
extern okapi::ControllerButton partner_X;
extern okapi::ControllerButton partner_Y;
extern okapi::ControllerButton partner_A;
extern okapi::ControllerButton partner_B;
extern okapi::ControllerButton partner_left;
extern okapi::ControllerButton partner_right;
extern okapi::ControllerButton partner_up;
extern okapi::ControllerButton partner_down;
extern okapi::ControllerButton master_R1;
extern okapi::ControllerButton master_R2;
extern okapi::ControllerButton master_L1;
extern okapi::ControllerButton master_L2;
extern okapi::ControllerButton master_X;
extern okapi::ControllerButton master_Y;
extern okapi::ControllerButton master_A;
extern okapi::ControllerButton master_B;
extern okapi::ControllerButton master_left;
extern okapi::ControllerButton master_right;

#endif  //  _GLOBAL_VARIABLES_HPP_
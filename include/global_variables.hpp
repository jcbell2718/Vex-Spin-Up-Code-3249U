#pragma once
#ifndef _GLOBAL_VARIABLES_HPP_
#define  _GLOBAL_VARIABLES_HPP_

#include "main.h"

extern pros::Mutex reset_mutex;
extern pros::Mutex target_mutex;
extern pros::Mutex vel_mutex;

extern std::string control_phase;

extern Chassis chassis;
extern Turret turret;
extern Intake intake;

extern pros::ADIDigitalOut expansion;
extern pros::Imu inertial;
extern pros::Gps gps;

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
extern okapi::ControllerButton master_up;
extern okapi::ControllerButton master_down;

#endif  //  _GLOBAL_VARIABLES_HPP_
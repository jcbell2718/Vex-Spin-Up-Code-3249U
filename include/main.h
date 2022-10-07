/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

/**
 * Global defines
 */
 # define PI 3.14159265358979323846
#define FRONT_LEFT_MOTOR_PORT 2
#define FRONT_RIGHT_MOTOR_PORT 1
#define BACK_LEFT_MOTOR_PORT 4
#define BACK_RIGHT_MOTOR_PORT 3
#define TURRET_MOTOR_PORT 5
#define FLYWHEEL_MOTOR_PORT 6
#define INERTIAL_PORT 19
#define GPS_PORT 20
#define CENTER_ENCODER_PORT_TOP 'A'
#define CENTER_ENCODER_PORT_BOTTOM 'B'
#define LEFT_ENCODER_PORT_TOP 'C'
#define LEFT_ENCODER_PORT_BOTTOM 'D'
#define RIGHT_ENCODER_PORT_TOP 'E'
#define RIGHT_ENCODER_PORT_BOTTOM 'F'


#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * Inter-File Global Variables
 */
extern double const launch_height;
extern double const launch_angle;
extern double const offset;
extern double const alpha;
extern double const launch_max_vel;
extern double const goal_height;
extern double const goal_x;
extern double const goal_y;
extern double const low_goal_x;
extern double const low_goal_y;
extern double const g;
extern double global_x_vel;
extern double global_y_vel;
extern bool auto_aim_enabled;
extern bool aiming_for_low_goal;
extern std::shared_ptr<okapi::OdomChassisController> chassis_controller;
extern std::shared_ptr<okapi::ChassisModel> chassis_model;
extern std::shared_ptr<okapi::AsyncMotionProfileController> chassis_profile_controller;
extern std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller;
extern std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller;
extern pros::Imu inertial;

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;
using namespace okapi::literals;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_

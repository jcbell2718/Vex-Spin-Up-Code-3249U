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
// #define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
// #define PROS_USE_LITERALS

/**
 * Global defines
 */
#define FRONT_LEFT_MOTOR_PORT 10
#define FRONT_RIGHT_MOTOR_PORT 9
#define BACK_LEFT_MOTOR_PORT 8
#define BACK_RIGHT_MOTOR_PORT 7
#define FLYWHEEL_MOTOR_PORT_1 4
#define FLYWHEEL_MOTOR_PORT_2 5
#define TURRET_MOTOR_PORT 3
#define INTAKE_MOTOR_PORT 2
#define ROLLER_OPTICAL_PORT 1
#define INERTIAL_PORT 20
#define GPS_PORT 11
#define LEFT_ENCODER_PORTS {6, 'E', 'F'}
#define CENTER_ENCODER_PORTS {6, 'C', 'D'}
#define RIGHT_ENCODER_PORTS {6, 'A', 'B'}
#define LIMIT_SWITCH_PORT 'A'
#define INDEXER_PORT 'H'
#define PTO_PORT 'E'
#define EXPANSION_PORT 'D'
#define EXPANSION_PORT_2 'G'


#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
#include "pros-grafana-lib/api.h"

#include "global_variables.hpp"
#include "general_functions.hpp"
#include "expansion.hpp"
#include "chassis.hpp"
#include "turret.hpp"
#include "intake.hpp"
#include "auton.hpp"
#include "grafana_setup.hpp"
#include "console_output.hpp"
#include "controller_lcd.hpp"
#include "initialization.hpp"

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

#pragma once
#ifndef _AIMING_FUNCTIONS_HPP_
#define  _AIMING_FUNCTIONS_HPP_

#include "main.h"

void velocity_recording_fn(void);
double angle_to_goal(double, double, double, double);
double ideal_velocity(double, double, double, double, double);
double trajectory_error(double, double, double, double, double, double);
double rotational_distance(double, double);
double unnormalized_rotation_to(double, double);
void auto_aim(void);

#endif  //  _AIMING_FUNCTIONS_HPP_

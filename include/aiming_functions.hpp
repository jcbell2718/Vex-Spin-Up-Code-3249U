#pragma once
#ifndef _AIMING_FUNCTIONS_HPP_
#define  _AIMING_FUNCTIONS_HPP_

#include "main.h"

double trajectory_error(double, double, double, double, double, double);
double ideal_velocity(double, double, double, double, double);
double angle_to_goal(double, double, double, double);
void auto_aim(void);

#endif  //  _AIMING_FUNCTIONS_HPP_

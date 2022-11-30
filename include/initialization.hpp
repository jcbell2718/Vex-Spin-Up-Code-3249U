#pragma once
#ifndef _INITIALIZATION_HPP_
#define  _INITIALIZATION_HPP_

#include "main.h"

void initialize(void);
double RobotXPosFeet(std::shared_ptr<okapi::OdomChassisController>);
double RobotYPosFeet(std::shared_ptr<okapi::OdomChassisController>);
double RobotAngleDegrees(std::shared_ptr<okapi::OdomChassisController>);
double TargetLaunchAngle(std::shared_ptr<okapi::AsyncPositionController<double, double>>);

#endif  //  _INITIALIZATION_HPP_
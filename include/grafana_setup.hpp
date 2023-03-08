#pragma once
#ifndef _GRAFANA_SETUP_HPP_
#define  _GRAFANA_SETUP_HPP_

#include "main.h"

double RobotXPosFeet(std::shared_ptr<okapi::OdomChassisController>);
double RobotYPosFeet(std::shared_ptr<okapi::OdomChassisController>);
double RobotAngleDegrees(std::shared_ptr<okapi::OdomChassisController>);
double TargetLaunchAngle(std::shared_ptr<okapi::AsyncPositionController<double, double>>);
extern std::shared_ptr<grafanalib::GUIManager> manager;

extern okapi::Motor grafana_flywheel_mtr_1;
extern okapi::Motor grafana_flywheel_mtr_2;
extern grafanalib::Variable<okapi::Motor> grafana_flywheel_mtr_1_var;
extern grafanalib::Variable<okapi::Motor> grafana_flywheel_mtr_2_var;
extern grafanalib::VariableGroup<okapi::Motor> grafana_flywheel_motor_vars;

void set_up_grafana();

#endif  //  _GRAFANA_SETUP_HPP_
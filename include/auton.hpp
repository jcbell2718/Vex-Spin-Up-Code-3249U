#pragma once
#include "okapi/api/units/QSpeed.hpp"
#ifndef _AUTON_HPP_
#define  _AUTON_HPP_

#include "main.h"


extern int auton_index;
extern std::vector<std::string> auton_list;
extern std::string auton;

void auton_setup();
void autonomous();

#endif  //  _AUTON_HPP_
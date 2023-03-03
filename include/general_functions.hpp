#pragma once
#ifndef _GENERAL_FUNCTIONS_HPP_
#define  _GENERAL_FUNCTIONS_HPP_

#include "main.h"

okapi::QAngle rotational_distance(okapi::QAngle target, okapi::QAngle current);
okapi::QAngle unnormalized_rotation_to(okapi::QAngle target, okapi::QAngle current);

#endif  //  _GENERAL_FUNCTIONS_HPP_
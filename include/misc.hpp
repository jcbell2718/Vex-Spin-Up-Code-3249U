#pragma once
#ifndef _MISC_HPP_
#define  _MISC_HPP_

#include "main.h"


okapi::QAngle rotational_distance(okapi::QAngle target, okapi::QAngle current);
okapi::QAngle unnormalized_rotation_to(okapi::QAngle target, okapi::QAngle current);


#endif  //  _MISC_HPP_
#pragma once
#ifndef _EXPANSION_HPP_
#define  _EXPANSION_HPP_

#include "api.h"

class Expansion {
    public:
        pros::ADIDigitalOut angry_birds;
        pros::ADIDigitalOut center_expansion;
        bool expanded;

        Expansion();
        void expand();
        void reset();
        void toggle();
};

extern Expansion expansion;

#endif  //  _EXPANSION_HPP_

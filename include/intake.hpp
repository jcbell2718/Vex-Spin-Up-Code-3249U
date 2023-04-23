#pragma once
#ifndef _INTAKE_HPP_
#define  _INTAKE_HPP_

#include "api.h"
#include "okapi/api.hpp"
#include "turret.hpp"

class Intake {
    public:
        bool intake_mode;
        bool indexing;
        bool turning_roller;
        std::string alliance_color;
        okapi::Motor intake_mtr;
        pros::Optical indexer_optical;
        pros::ADIDigitalIn disk_switch;
        pros::ADIDigitalOut indexer;
        pros::ADIDigitalOut PTO;
        pros::Optical roller_optical;

        Intake();
        bool turn_roller();
        void index();
        void PTO_to_intake();
        void PTO_to_roller_mech();
        bool disk_on_indexer();
};

extern Intake intake;
void intake_regulation_fn();

#endif  //  _INTAKE_HPP_
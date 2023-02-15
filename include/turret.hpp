#pragma once
#ifndef _TURRET_HPP_
#define  _TURRET_HPP_

#include "api.h"
#include "okapi/api.hpp"
#include "chassis.hpp"

class Turret {
    public:
        okapi::Motor turret_mtr;
        okapi::Motor flywheel_mtr_1;
        okapi::Motor flywheel_mtr_2;
        okapi::MotorGroup flywheel_mtrs;
        // Okapi control groups
        std::shared_ptr<okapi::AsyncPositionController<double, double> > turret_controller;
        std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller;
        // Constants
        okapi::QLength launch_height;
        okapi::QAngle launch_angle;
        okapi::QLength offset;
        okapi::QAngle alpha;
        okapi::QLength goal_height;
        okapi::QAcceleration g;
        // Variables
        bool auto_aim_enabled;
        bool aiming_for_low_goal;
        okapi::QLength opponent_goal_x;
        okapi::QLength opponent_goal_y;
        okapi::QLength alliance_goal_x;
        okapi::QLength alliance_goal_y;
        okapi::QLength x_pos;
        okapi::QLength y_pos;
        okapi::QSpeed x_vel;
        okapi::QSpeed y_vel;
        okapi::QLength target_x;
        okapi::QLength target_y;
        okapi::QLength target_height;
        okapi::QAngle turret_absolute_angle;
        okapi::QSpeed launch_velocity;
        double launch_RPM;

        Turret();
        okapi::QAngle angle_to_target();
        okapi::QSpeed ideal_velocity();
        okapi::QLength trajectory_error(okapi::QAngle theta, okapi::QSpeed launch_vel);
        void target_low_goal();
        void target_high_goal();
        void switch_alliance();
        void update_position(Chassis chassis);
        void set_target_angle(okapi::QAngle angle);
        void set_target_velocity(okapi::QSpeed velocity);
        void set_target_RPM(double rpm);
};

void auto_aim();

#endif  //  _TURRET_HPP_

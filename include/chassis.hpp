#pragma once
#ifndef _CHASSIS_HPP_
#define  _CHASSIS_HPP_

#include "api.h"
#include "okapi/api.hpp"

class Chassis {
    public:
        okapi::QLength x_pos;
        okapi::QLength y_pos;
        okapi::QSpeed x_vel;
        okapi::QSpeed y_vel;
        okapi::QAngle angle;
        bool using_gps;
        // Chassis
        okapi::Motor front_left_mtr;
        okapi::Motor front_right_mtr;
        okapi::Motor back_left_mtr;
        okapi::Motor back_right_mtr;
        okapi::ADIEncoder center_encoder;
        okapi::ADIEncoder left_encoder;
        okapi::ADIEncoder right_encoder;
        std::shared_ptr<okapi::OdomChassisController> controller;
        std::shared_ptr<okapi::XDriveModel> model;
        std::shared_ptr<okapi::AsyncMotionProfileController> profile_controller;
        double max_vel;

        Chassis();
        void drive_to_PD(okapi::QLength x, okapi::QLength y, okapi::QAngle theta);
        void drive_to_default_odom(okapi::QLength x, okapi::QLength y);
        void drive_raw(double forward, double strafe, double yaw);
        void update_position();
        void update_position_gps();
        void set_position(okapi::QLength x, okapi::QLength y, okapi::QAngle theta);
        void align_odometry_position(int x, int y);
        void align_odometry_orientation(int x, int y);
        void reset_encoders();
};

void velocity_recording_fn();

#endif  //  _CHASSIS_HPP_

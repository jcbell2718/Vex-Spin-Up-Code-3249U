#include "global_variables.hpp"
#include "main.h"

// Mutexes
pros::Mutex reset_mutex;
pros::Mutex target_mutex;
pros::Mutex vel_mutex;

// Global Variables
std::string control_phase = "initialization";

// Pointers to subsystems
// The subsystems are weird when declared before initialize, but this provides a way for global access
Chassis* chassis_pointer;
Turret* turret_pointer;
Intake* intake_pointer;

// Expansion
pros::ADIDigitalOut expansion(EXPANSION_PORT);

// Other
pros::Imu inertial = pros::Imu(INERTIAL_PORT);
pros::Gps gps(GPS_PORT, 0, 0, 0);

// Controllers
okapi::Controller master(okapi::ControllerId::master);
okapi::Controller partner(okapi::ControllerId::partner);
okapi::ControllerButton partner_R1(okapi::ControllerId::partner, okapi::ControllerDigital::R1);
okapi::ControllerButton partner_R2(okapi::ControllerId::partner, okapi::ControllerDigital::R2);
okapi::ControllerButton partner_L1(okapi::ControllerId::partner, okapi::ControllerDigital::L1);
okapi::ControllerButton partner_L2(okapi::ControllerId::partner, okapi::ControllerDigital::L2);
okapi::ControllerButton partner_X(okapi::ControllerId::partner, okapi::ControllerDigital::X);
okapi::ControllerButton partner_Y(okapi::ControllerId::partner, okapi::ControllerDigital::Y);
okapi::ControllerButton partner_A(okapi::ControllerId::partner, okapi::ControllerDigital::A);
okapi::ControllerButton partner_B(okapi::ControllerId::partner, okapi::ControllerDigital::B);
okapi::ControllerButton partner_left(okapi::ControllerId::partner, okapi::ControllerDigital::left);
okapi::ControllerButton partner_right(okapi::ControllerId::partner, okapi::ControllerDigital::right);
okapi::ControllerButton partner_up(okapi::ControllerId::partner, okapi::ControllerDigital::up);
okapi::ControllerButton partner_down(okapi::ControllerId::partner, okapi::ControllerDigital::down);
okapi::ControllerButton master_R1(okapi::ControllerId::master, okapi::ControllerDigital::R1);
okapi::ControllerButton master_R2(okapi::ControllerId::master, okapi::ControllerDigital::R2);
okapi::ControllerButton master_L1(okapi::ControllerId::master, okapi::ControllerDigital::L1);
okapi::ControllerButton master_L2(okapi::ControllerId::master, okapi::ControllerDigital::L2);
okapi::ControllerButton master_X(okapi::ControllerId::master, okapi::ControllerDigital::X);
okapi::ControllerButton master_Y(okapi::ControllerId::master, okapi::ControllerDigital::Y);
okapi::ControllerButton master_A(okapi::ControllerId::master, okapi::ControllerDigital::A);
okapi::ControllerButton master_B(okapi::ControllerId::master, okapi::ControllerDigital::B);
okapi::ControllerButton master_left(okapi::ControllerId::master, okapi::ControllerDigital::left);
okapi::ControllerButton master_right(okapi::ControllerId::master, okapi::ControllerDigital::right);
okapi::ControllerButton master_up(okapi::ControllerId::master, okapi::ControllerDigital::left);
okapi::ControllerButton master_down(okapi::ControllerId::master, okapi::ControllerDigital::right);
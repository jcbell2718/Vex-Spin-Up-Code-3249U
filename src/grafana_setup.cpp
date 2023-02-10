#include "main.h"
#include "global_variables.hpp"

// Grafana conversion functions:
double RobotXPosFeet(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().x.convert(okapi::foot);}
double RobotYPosFeet(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().y.convert(okapi::foot);}
double RobotAngleDegrees(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().theta.convert(okapi::degree);}
double TargetLaunchAngle(std::shared_ptr<okapi::AsyncPositionController<double, double>> turret) {return fmod(-turret -> getTarget() - inertial.get_yaw() + 9000.*360., 360.);}

// Manager in extended scope
std::shared_ptr<grafanalib::GUIManager> manager;

void set_up_grafana() {
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
    manager = std::make_shared<grafanalib::GUIManager>();
	manager->setRefreshRate(200); // Hopefully sufficient for wireless

	grafanalib::Variable<okapi::Motor> front_left_mtr_var("Front Left Motor", chassis.front_left_mtr);
	grafanalib::Variable<okapi::Motor> front_right_mtr_var("Front Right Motor", chassis.front_right_mtr);
	grafanalib::Variable<okapi::Motor> back_left_mtr_var("Back Left Motor", chassis.back_left_mtr);
	grafanalib::Variable<okapi::Motor> back_right_mtr_var("Back Right Motor", chassis.back_right_mtr);
	grafanalib::VariableGroup<okapi::Motor> chassis_motor_vars({front_left_mtr_var, front_right_mtr_var, back_left_mtr_var, back_right_mtr_var});
	chassis_motor_vars.add_getter("Temperature", &okapi::Motor::getTemperature);
	manager->registerDataHandler(&chassis_motor_vars);

	grafanalib::Variable<okapi::Motor> flywheel_mtr_1_var("Flywheel Motor 1", turret.flywheel_mtr_1);
	grafanalib::Variable<okapi::Motor> flywheel_mtr_2_var("Flywheel Motor 2", turret.flywheel_mtr_2);
	grafanalib::VariableGroup<okapi::Motor> flywheel_motor_vars({flywheel_mtr_1_var, flywheel_mtr_2_var});
	flywheel_motor_vars.add_getter("Temperature", &okapi::Motor::getTemperature);
	flywheel_motor_vars.add_getter("Power", &okapi::Motor::getPower);
	manager->registerDataHandler(&flywheel_motor_vars);

	grafanalib::Variable<std::shared_ptr<okapi::AsyncVelocityController<double, double>>> flywheel_controller_var("Flywheel Controller", turret.flywheel_controller);
	flywheel_controller_var.add_getter("Flywheel Target RPM", &okapi::AsyncPositionController<double, double>::getTarget);
	manager->registerDataHandler(&flywheel_controller_var);

	grafanalib::Variable<std::shared_ptr<okapi::AsyncPositionController<double, double>>> turret_controller_var("Turret Controller", turret.turret_controller);
	turret_controller_var.add_getter("Target Launch Angle", &TargetLaunchAngle);
	manager->registerDataHandler(&turret_controller_var);

	grafanalib::Variable<std::shared_ptr<okapi::OdomChassisController>> chassis_controller_var("Chassis Controller", chassis.controller);
	chassis_controller_var.add_getter("X Position", &RobotXPosFeet);
	chassis_controller_var.add_getter("Y Position", &RobotYPosFeet);
	chassis_controller_var.add_getter("Orientation", &RobotAngleDegrees);
	manager->registerDataHandler(&chassis_controller_var);

	manager->startTask();
}
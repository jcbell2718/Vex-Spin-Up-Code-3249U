#include "main.h"
#include "global_variables.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "pros/misc.h"

// Grafana conversion functions:
double RobotXPosFeet(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().x.convert(okapi::foot);}
double RobotYPosFeet(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().y.convert(okapi::foot);}
double RobotAngleDegrees(std::shared_ptr<okapi::OdomChassisController> chassis) {return chassis -> getState().theta.convert(okapi::degree);}
double TargetLaunchAngle(std::shared_ptr<okapi::AsyncPositionController<double, double>> turret) {return fmod(-turret -> getTarget() - inertial.get_yaw() + 9000.*360., 360.);}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Reset Quad Encoders from previous run of code
	center_encoder.reset();
	left_encoder.reset();
	right_encoder.reset();
	pros::delay(500);

	// Model Initialization
	chassis_controller = okapi::ChassisControllerBuilder()
		.withMotors(front_left_mtr, front_right_mtr, back_right_mtr, back_left_mtr)
		.withSensors(left_encoder, right_encoder, center_encoder)
		// Green gearset, 36/84 gear ratio, 3.25 inch diameter wheels, 16.4 inch wheel track
		.withDimensions({okapi::AbstractMotor::gearset::red, (36./84.)}, {{3.25_in, 15._in + 15._in/16.}, okapi::imev5RedTPR})
		// tracking wheel diameter | wheel track | middle encoder distance
		.withOdometry({{3.25_in, 9.75_in, 1.875_in, 3.25_in}, okapi::quadEncoderTPR})
		.notParentedToCurrentTask()
		.buildOdometry();
	chassis_model = std::dynamic_pointer_cast<okapi::XDriveModel>(chassis_controller -> getModel());
	chassis_profile_controller = okapi::AsyncMotionProfileControllerBuilder()
		.withOutput(chassis_controller)
		// Max speed 3.66 m/s, max acceleration 3 m/s^2, max jerk 1000 m/s^3
		.withLimits({3.66, 3, 1000})
		.withLogger(okapi::Logger::getDefaultLogger())
		.notParentedToCurrentTask()
		.buildMotionProfileController();
	chassis_max_vel = chassis_model -> getMaxVelocity();
	turret_controller = okapi::AsyncPosControllerBuilder()
		.withMotor(turret_mtr)
		.withSensor(okapi::IntegratedEncoder(turret_mtr))
		.withGearset({okapi::AbstractMotor::gearset::green, (148./36.)})
		.withLogger(okapi::Logger::getDefaultLogger())
		.notParentedToCurrentTask()
		.build();
	flywheel_controller = okapi::AsyncVelControllerBuilder()
		.withMotor(flywheel_mtrs)
		.withSensor(okapi::IntegratedEncoder(flywheel_mtr_1))
		// Not actually, but equivalent. It's really 18:30 3600 rpm cartridge
		.withGearset({okapi::AbstractMotor::gearset::blue, (18./(6.*30.))})
		.withLogger(okapi::Logger::getDefaultLogger())
		.notParentedToCurrentTask()
		.build();
	
	// Grafana
	auto manager = std::make_shared<grafanalib::GUIManager>();
	manager->setRefreshRate(150); // Hopefully sufficient for wireless

	grafanalib::Variable<okapi::Motor> front_left_mtr_var("Front Left Motor", front_left_mtr);
	grafanalib::Variable<okapi::Motor> front_right_mtr_var("Front Right Motor", front_right_mtr);
	grafanalib::Variable<okapi::Motor> back_left_mtr_var("Back Left Motor", back_left_mtr);
	grafanalib::Variable<okapi::Motor> back_right_mtr_var("Back Right Motor", back_right_mtr);
	grafanalib::VariableGroup<okapi::Motor> chassis_motor_vars({front_left_mtr_var, front_right_mtr_var, back_left_mtr_var, back_right_mtr_var});
	chassis_motor_vars.add_getter("Temperature", &okapi::Motor::getTemperature);
	manager->registerDataHandler(&chassis_motor_vars);

	grafanalib::Variable<okapi::Motor> flywheel_mtr_1_var("Flywheel Motor 1", flywheel_mtr_1);
	grafanalib::Variable<okapi::Motor> flywheel_mtr_2_var("Flywheel Motor 2", flywheel_mtr_2);
	grafanalib::VariableGroup<okapi::Motor> flywheel_motor_vars({flywheel_mtr_1_var, flywheel_mtr_2_var});
	flywheel_motor_vars.add_getter("Temperature", &okapi::Motor::getTemperature);
	flywheel_motor_vars.add_getter("Power", &okapi::Motor::getPower);
	manager->registerDataHandler(&flywheel_motor_vars);

	grafanalib::Variable<std::shared_ptr<okapi::AsyncVelocityController<double, double>>> flywheel_controller_var("Flywheel Controller", flywheel_controller);
	flywheel_controller_var.add_getter("Flywheel Target RPM", &okapi::AsyncPositionController<double, double>::getTarget);
	manager->registerDataHandler(&flywheel_controller_var);

	grafanalib::Variable<std::shared_ptr<okapi::AsyncPositionController<double, double>>> turret_controller_var("Turret Controller", turret_controller);
	turret_controller_var.add_getter("Target Launch Angle", &TargetLaunchAngle);
	manager->registerDataHandler(&turret_controller_var);

	grafanalib::Variable<std::shared_ptr<okapi::OdomChassisController>> chassis_controller_var("Chassis Controller", chassis_controller);
	chassis_controller_var.add_getter("X Position", &RobotXPosFeet);
	chassis_controller_var.add_getter("Y Position", &RobotYPosFeet);
	chassis_controller_var.add_getter("Orientation", &RobotAngleDegrees);
	manager->registerDataHandler(&chassis_controller_var);

	manager->startTask();

	bool selection_made ;
	std::string auton_text;
	std::string color_text;

	// Initialize tasks
	pros::Task velocity_recording(velocity_recording_fn);
	pros::Task auto_aiming(auto_aim);
	pros::Task intake_control(intake_regulation_function);

	// Setup selection is in loop so autons can be run repeatedly without turning off the program
	// DON'T TURN ON THE ROBOT PLUGGED INTO FIELD CONTROL IMMEDIATELY
	while(!pros::competition::is_connected()) {
		// Disables auto-aim and the intake so they don't run during setup selection
		auto_aim_enabled = false;
		intake_enabled = false;

		// Auton selection menu through the controller
		selection_made = false;
		auton_text = auton + "          ";
		master.setText(0, 0, "Selecting Auton:   ");
		pros::delay(120);
		master.setText(1, 0, auton.c_str());
		pros::delay(120);
		while(selection_made == false) {
			if(master_A.changedToPressed()) {
				// Submit choice
				selection_made = true;
			} else if(master_left.changedToPressed()) {
				// Cycle through choices backwards
				auton_index--;
				if(auton_index < 0) auton_index = auton_list.size() - 1;
				auton = auton_list[auton_index];
				auton_text = auton + "          ";
				master.setText(1, 0, auton_text);
				pros::delay(120);
			} else if(master_right.changedToPressed()) {
				// Cycle through choices forwards
				auton_index++;
				if(auton_index == auton_list.size()) auton_index = 0;
				auton = auton_list[auton_index];
				auton_text = auton + "          ";
				master.setText(1, 0, auton_text);
				pros::delay(120);
			}
			pros::delay(20);
		}

		// Alliance selection menu through the controller
		selection_made = false;
		color_text = alliance_color + "          ";
		master.setText(0, 0, "Selecting Alliance:   ");
		pros::delay(120);
		master.setText(1, 0, color_text);
		pros::delay(120);
		while(selection_made == false) {
			if(master_A.changedToPressed()) {
				// Submit choice
				selection_made = true;
			} else if(master_left.changedToPressed() || master_right.changedToPressed()) {
				// Cycle through choices
				if(alliance_color == "red") alliance_color = "blue";
				else alliance_color = "red";
				color_text = alliance_color + "          ";
				master.setText(1, 0, color_text);
				pros::delay(120);
			} 
			pros::delay(20);
		}

		// Initialize setup dependent on auton choice
		chassis_model -> resetSensors();
		turret_controller -> tarePosition();
		inertial.tare();
		pros::delay(120);
		if(auton == "PD Tuning") {
			chassis_controller -> setState({0_ft, 0_ft, 0_deg});
		} else if(auton == "Odometry Tuning") {
			chassis_controller -> setState({0_ft, 0_ft, 0_deg});
		} else if(auton == "Roller Start Double Sweep") {
			chassis_controller -> setState({1_ft, -9_ft, 0_deg});
		} else if(auton == "Roller Only") {
			chassis_controller -> setState({1_ft, -9_ft, 0_deg});
		}

		// Displays final configuration
		master.setText(0, 0, auton_text);
		pros::delay(120);
		master.setText(2, 0, "  -Setup Loaded-  ");
		pros::delay(120);

		// Auton or opcontrol can be enabled without competition switch
		// Also holds to prevent automatic opcontrol when wanting to plug into field control
		while(!pros::competition::is_connected()) {
			if(master_Y.changedToPressed()) {
				autonomous();
				break;
			} else if(master_A.changedToPressed()) {
				opcontrol();
				break;
			} else if(master_B.changedToPressed()) {
				break;
			}
			pros::delay(20);
		}
	}
}
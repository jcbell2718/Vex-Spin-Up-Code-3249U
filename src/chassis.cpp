#include "main.h"

Chassis::Chassis() :
    x_pos(0._ft),
    y_pos(0._ft),
    x_vel(0._mps),
    y_vel(0._mps),
    angle(0._deg),
    using_gps(false),
    front_left_mtr(FRONT_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations),
    front_right_mtr(FRONT_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations),
    back_left_mtr(BACK_LEFT_MOTOR_PORT, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations),
    back_right_mtr(BACK_RIGHT_MOTOR_PORT, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations),
    center_encoder(CENTER_ENCODER_PORTS, false),
    left_encoder(LEFT_ENCODER_PORTS, false),
    right_encoder(RIGHT_ENCODER_PORTS, true),
    max_vel()
{
    reset_encoders();
    controller = okapi::ChassisControllerBuilder()
		.withMotors(front_left_mtr, front_right_mtr, back_right_mtr, back_left_mtr)
		.withSensors(left_encoder, right_encoder, center_encoder)
		// Gearset | gear ratio | wheel diameter | wheel track (driving) | TPR
		.withDimensions({okapi::AbstractMotor::gearset::green, (1./1.)}, {{3.25_in, 15._in + 15._in/16.}, okapi::imev5GreenTPR})
		// Tracking wheel diameter | wheel track (tracking) | middle encoder distance | center tracking wheel diameter
		.withOdometry({{2.75_in, 13._in + 15._in/16., 5.5_in, 2.75_in}, okapi::quadEncoderTPR})
		.notParentedToCurrentTask()
		.buildOdometry();
    controller->setState({x_pos, -y_pos, angle});
	model = std::dynamic_pointer_cast<okapi::XDriveModel>(controller -> getModel());
	profile_controller = okapi::AsyncMotionProfileControllerBuilder()
		.withOutput(controller)
		// Max speed (m/s) | max acceleration (m/s^2) | max jerk (m/s^3)
		.withLimits({3.66, 3, 1000})
		.withLogger(okapi::Logger::getDefaultLogger())
		.notParentedToCurrentTask()
		.buildMotionProfileController();
	max_vel = model -> getMaxVelocity();
}

void Chassis::drive_to_PD(okapi::QLength x, okapi::QLength y, okapi::QAngle theta) {
    // Drives to position on field using PD control
    // Inputs treat home goal corner as bottom left of first quadrant of coordinate plane
    // Input rotations are counterclockwise from x-axis
    // Convert from QLengths to doubles
    double target_x = x.convert(okapi::inch);
    double target_y = -y.convert(okapi::inch); // Convert to inverted okapi y-direction
    okapi::QAngle target_theta = -theta; // Convert to default okapi clockwise angle
    // PD parameters:
    double pkP = .1;
    double pkD = 0;
    double akP = 0;
    double akD = 0;
    int delay = 20;
    int stability_counter = 0;
    double a_error = rotational_distance(target_theta, controller -> getState().theta).convert(okapi::degree);
    double x_error = target_x + controller -> getState().x.convert(okapi::inch);
    double y_error = target_y - controller -> getState().y.convert(okapi::inch);
    double a_error_previous;
    double x_error_previous;
    double y_error_previous;
    double a_error_derivative;
    double x_error_derivative;
    double y_error_derivative;
    double unadjusted_x_input;
    double unadjusted_y_input;
    double x_input;
    double y_input;
    double input_scaler;
    double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    while(stability_counter < 15) {
        a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        a_error = rotational_distance(target_theta, controller -> getState().theta).convert(okapi::degree);
        x_error = target_x - controller -> getState().x.convert(okapi::inch);
        y_error = target_y - controller -> getState().y.convert(okapi::inch);
        total_error = abs(a_error) + abs(x_error) + abs(y_error);
        a_error_derivative = (a_error - a_error_previous)/(delay/1000.);
        x_error_derivative = (x_error - x_error_previous)/(delay/1000.);
        y_error_derivative = (y_error - y_error_previous)/(delay/1000.);
        if(total_error < 5) stability_counter++; // Counter to determine if the robot has settled
        else stability_counter = 0;
        // Adjustment to make path straight when not at 45 degrees at higher than max speed in one direction
        unadjusted_x_input = pkP*x_error + pkD*x_error_derivative;
        unadjusted_y_input = pkP*y_error + pkD*y_error_derivative;
        // if(unadjusted_x_input > 1 || unadjusted_y_input > 1) input_scaler = std::max(unadjusted_x_input, unadjusted_y_input);
        // else input_scaler = 1;
        // x_input = unadjusted_x_input/input_scaler;
        // y_input = unadjusted_y_input/input_scaler;
        model -> fieldOrientedXArcade(unadjusted_x_input, unadjusted_y_input, akP*a_error + akD*a_error_derivative, controller -> getState().theta);
        pros::delay(delay);
    }
}

void Chassis::drive_to_default_odom(okapi::QLength x, okapi::QLength y) {
    controller -> driveToPoint({x, -y});
}

void Chassis::drive_raw(double forward, double strafe, double yaw) {
    model -> xArcade(forward, strafe, yaw);
}

void Chassis::update_position() {
    reset_mutex.take();
    x_pos = controller -> getState().x;
    y_pos = -controller -> getState().y;
    angle = -controller -> getState().theta;
    reset_mutex.give();
}

void Chassis::update_position_gps() {
    reset_mutex.take();
    controller -> setState({gps.get_status().x*okapi::millimeter,gps.get_status().y*okapi::millimeter,gps.get_status().yaw*okapi::degree});
    x_pos = controller -> getState().x;
    y_pos = -controller -> getState().y;
    angle = -controller -> getState().theta;
    reset_mutex.give();
}

void Chassis::set_position(okapi::QLength x, okapi::QLength y, okapi::QAngle theta) {
    reset_mutex.take();
    x_pos = x;
    y_pos = y;
    angle = theta;
    controller -> setState({x, -y, -theta});
    reset_mutex.give();
}

void Chassis::align_odometry_position(int x, int y) {
    reset_mutex.take();
    if(x > 0) {
        controller -> setState({12_ft-9_in, controller -> getState().y, controller -> getState().theta});
    } else if(x < 0) {
        controller -> setState({9_in, controller -> getState().y, controller -> getState().theta});
    }
    if(y > 0) {
        controller -> setState({controller -> getState().x, -(12_ft-9_in), controller -> getState().theta});
    } else if(y < 0) {
        controller -> setState({controller -> getState().x, -9_in, controller -> getState().theta});
    }
    reset_mutex.give();
    // AFTER reset mutex is already given up, might get stuck otherwise
    update_position();
}

void Chassis::align_odometry_orientation(int x, int y) {
    reset_mutex.give();
    controller -> setState({controller -> getState().x, controller -> getState().y, -atan2(x, y)*okapi::degree});
    reset_mutex.give();
    // AFTER reset mutex is already given up, might get stuck otherwise
    update_position();
}

void Chassis::reset_encoders() {
    center_encoder.reset();
	left_encoder.reset();
	right_encoder.reset();
	pros::delay(500);
}

void velocity_recording_fn() {
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
	// Records robot velocity globally, although specific values are frozen during computations to avoid strange behavior
    chassis.update_position();
	okapi::QLength new_x_pos = chassis.x_pos;
	okapi::QLength new_y_pos = chassis.y_pos;
	okapi::QLength old_x_pos;
	okapi::QLength old_y_pos;
    okapi::AverageFilter<3> vel_filter;
    okapi::Timer timer;
    timer.getDt();
    okapi::QTime delay;
	while(true) {
        pros::delay(20);
		old_x_pos = new_x_pos;
		old_y_pos = new_y_pos;
	    if(chassis.using_gps) chassis.update_position_gps(); else chassis.update_position();
        new_x_pos = chassis.x_pos;
		new_y_pos = chassis.y_pos;
        vel_mutex.take();
        delay = timer.getDt();
		chassis.x_vel = vel_filter.filter((new_x_pos-old_x_pos).convert(okapi::inch))*okapi::inch/(delay);
		chassis.y_vel = vel_filter.filter((new_y_pos-old_y_pos).convert(okapi::inch))*okapi::inch/(delay);
        vel_mutex.give();
	}
}
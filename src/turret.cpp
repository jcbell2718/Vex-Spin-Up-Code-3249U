#include "main.h"

Turret::Turret() : 
    turret_mtr(TURRET_MOTOR_PORT, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees),
    flywheel_mtr_1(FLYWHEEL_MOTOR_PORT_1, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations),
    flywheel_mtr_2(FLYWHEEL_MOTOR_PORT_2, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations),
    flywheel_mtrs({flywheel_mtr_1, flywheel_mtr_2}),
    launch_height(15.5_in),
    launch_angle(32._deg),
    chassis_offset(1.5_in),
    turret_offset(5.6_in),
    alpha(-51.34_deg),
    goal_height(30._in),
    g(1._G),
    auto_aim_enabled(false),
    aiming_for_low_goal(false),
    opponent_goal_x(141.25_in - (123.75_in)),
    opponent_goal_y(141.25_in - (123.75_in)),
    alliance_goal_x(123.75_in),
    alliance_goal_y(123.75_in),
    orientation(0._deg),
    x_pos(0._ft),
    y_pos(0._ft),
    x_vel(0._mps),
    y_vel(0._mps),
    target_x(10.5_ft),
    target_y(10.5_ft),
    target_height(30._in),
    turret_absolute_angle(0_deg),
    launch_velocity(0_mps),
    launch_RPM(0)
{}

void Turret::build_models() {
    // Initializes the okapi controllers
    turret_controller = okapi::AsyncPosControllerBuilder()
        .withMotor(turret_mtr)
        .withSensor(okapi::IntegratedEncoder(turret_mtr))
        // Gearset | gear ratio
        .withGearset({okapi::AbstractMotor::gearset::blue, (148./12.)})
        .withGains({0.003,0.,0.})
        .withLogger(okapi::Logger::getDefaultLogger())
        .notParentedToCurrentTask()
        .build();
    flywheel_controller = okapi::AsyncVelControllerBuilder()
        .withMotor(flywheel_mtrs)
        .withSensor(okapi::IntegratedEncoder(flywheel_mtr_1))
        .withGearset({okapi::AbstractMotor::gearset::blue, (64./(6.*84.))}) // 64:84 3600 rpm cartridge
        // .withGains({0.,0.,0.001,0.})
        // .withVelMath(okapi::VelMathFactory::createPtr(okapi::imev5BlueTPR * 64./(6.*84.))) // Ticks per WHAT revolution? Motor or end result?
        .withLogger(okapi::Logger::getDefaultLogger())
        .notParentedToCurrentTask()
        .build();
    set_target_RPM(0);
    set_target_angle(0_deg);
    std::cout << turret_absolute_angle.convert(okapi::degree) << std::endl;
}

okapi::QAngle Turret::angle_to_target() {
    // Calculates the (counterclockwise) angle to the target
    return atan2((target_y - y_pos).convert(okapi::inch), (target_x - x_pos).convert(okapi::inch))*okapi::radian;
}

okapi::QSpeed Turret::ideal_velocity() {
    // Calculates the ideal launch velocity assuming the robot wasn't moving
    // For this I'm using a 2D, simplified model
    // y_vel = v_0*sin(phi)
    // x_vel = v_0*cos(phi)
    // T = d/(v_0*cos(phi))
    // h_goal = T*v_0*sin(phi) + 1/2*g*T^2 + h_launcher
    // h_goal = d/(v_0*cos(phi))*v_0*sin(phi) + 1/2*g*d^2/(v_0*cos(phi))^2 + h_launcher
    // h_goal = d/cos(phi)*sin(phi) + 1/2*g*d^2/(v_0*cos(phi))^2 + h_launcher
    // h_goal = d*tan(phi) + g*d^2/(2*v_0^2*cos(phi)^2) + h_launcher
    // h_goal - h_launcher - d*tan(phi) = g*d^2/(2*v_0^2*cos(phi)^2)
    // 1/v_0^2 = (2*cos(phi)^2)(h_goal - h_launcher - d*tan(phi))/(g*d^2)
    // v_0^2 = (g*d^2)/((2*cos(phi)^2)(h_goal - h_launcher - d*tan(phi))
    // v_0 = sqrt((g*d^2)/((2*cos(phi)^2)(h_goal - h_launcher - d*tan(phi)))
    okapi::QLength dist = sqrt(square(target_x-x_pos) + square(target_y-y_pos));
    auto root = g*square(dist)/((2*square(cos(launch_angle)))*(target_height - launch_height - dist*tan(launch_angle)));
    // Complex root -> infinite velocity+ to hit target (without considering bot movement - still maybe possible)
    if(root <= square(0_mps)) return 320_in/1_s;
    okapi::QSpeed launch_vel = sqrt(root);
    return launch_vel;
}

okapi::QLength Turret::trajectory_error(okapi::QAngle theta, okapi::QSpeed launch_vel) {
    // Calculates the distance in a given trajectory from the ideal end point
    //
    //       [|v_0|cos(phi)cos(theta)]
    // v_0 = [|v_0|cos(phi)sin(theta)]
    //       [     |v_0|sin(phi)     ]
    //
    // h_t = h_launcher + |v_0|sin(phi)t - g/2*t^2
    // 0 = (h_launcher - h_goal) + |v_0|sin(phi)t - g/2*t^2
    // Then use the quadratic formula...
    auto discriminant = square(launch_vel*sin(launch_angle)) - 4.*(launch_height - goal_height)*(-g/2.);
    if(discriminant < square(0_mps)) return 9001_in; // Complex root -> doesn't reach target height
    okapi::QTime t = -(-launch_vel*sin(launch_angle) - sqrt(discriminant))/g;
    okapi::QLength x_end_pos = x_pos + turret_offset*cos(theta + alpha) + t*launch_vel*cos(launch_angle)*cos(theta) + t*x_vel;
    okapi::QLength y_end_pos = y_pos + turret_offset*sin(theta + alpha) + t*launch_vel*cos(launch_angle)*sin(theta) + t*y_vel;
    okapi::QLength error = sqrt(square(target_x-x_end_pos) + square(target_y-y_end_pos));
    return error;
}

void Turret::target_low_goal() {
    target_mutex.take();
    aiming_for_low_goal = true;
    target_x = opponent_goal_x;
    target_y = opponent_goal_y;
    target_height = 0_in;
    target_mutex.give();
}

void Turret::target_high_goal() {
    target_mutex.take();
    aiming_for_low_goal = false;
    target_x = alliance_goal_x;
    target_y = alliance_goal_y;
    target_height = goal_height;
    target_mutex.give();
}

void Turret::switch_alliance() {
    okapi::QLength temp_x = alliance_goal_x;
    okapi::QLength temp_y = alliance_goal_y;
    alliance_goal_x = opponent_goal_x;
    alliance_goal_y = opponent_goal_y;
    opponent_goal_x = temp_x;
    opponent_goal_y = temp_y;
    if(aiming_for_low_goal) target_low_goal();
    else target_high_goal();
}

void Turret::update_position(Chassis chassis) {
    reset_mutex.take();
    orientation = -chassis.controller -> getState().theta;
    x_pos = chassis.controller -> getState().x + cos(orientation)*chassis_offset;
    y_pos = -chassis.controller -> getState().y + sin(orientation)*chassis_offset;
    reset_mutex.give();
    vel_mutex.take();
    x_vel = chassis.x_vel;
    y_vel = chassis.y_vel;
    vel_mutex.give();
}

void Turret::set_target_angle(okapi::QAngle angle) {
    turret_absolute_angle = angle;
    turret_controller -> setTarget(angle.convert(okapi::degree)); 
}

void Turret::set_target_velocity(okapi::QSpeed velocity) {
    launch_velocity = velocity;
    launch_RPM = (launch_velocity*2./(3._in*okapi::pi)*1._min).getValue()*1.1; // ((in/min)*(edge speed/resulting speed))*(rot/in)*1 minute
    flywheel_controller -> setTarget(launch_RPM);
}

void Turret::set_target_RPM(double rpm) {
    launch_RPM = rpm;
    launch_velocity = (launch_RPM*3._in*okapi::pi)/(2.*1._min);
    flywheel_controller -> setTarget(launch_RPM);
}

Turret turret = Turret();

void auto_aim() {
    // When enabled, automatically adjusts turret position and flywheel speed to target the goal
    okapi::QAngle launch_theta;
    okapi::QSpeed launch_vel;
    okapi::QLength error_0;
    okapi::QLength error;
    okapi::QLength error_dv;
    okapi::QLength error_dt;
    int depth;
    bool oscillating;
    double r;
    while(true) {
        if(turret.auto_aim_enabled) {
            if(turret.turret_controller -> isDisabled()) turret.turret_controller -> flipDisable();
            turret.update_position(chassis);
            target_mutex.take();
            if(turret.aiming_for_low_goal) {
                // Assumes ideal values are true when aiming for the low goal, since accuracy is less important
                launch_theta = turret.angle_to_target() - (45_deg * 1.45_m/turret.ideal_velocity() / okapi::second);
                launch_vel = 1.15 * turret.ideal_velocity();
            } else {
                // Determines best launch parameters for high goal using gradient descent
                // Uses inaccurate estimation as starting point
                launch_theta = turret.angle_to_target();
                launch_vel = turret.ideal_velocity();
                error = 9000_in;
                error_0 = 9000_in;
                depth = 0;
                // Initial coefficient for gradient descent - good for low curvature
                r = 5.;
                oscillating = false;
                // Calculate initial error outside loop to remove need to store past launch parameters
                error = turret.trajectory_error(launch_theta, launch_vel);
                error_dv = turret.trajectory_error(launch_theta, launch_vel + .01*1_in/1_s);
                error_dt = turret.trajectory_error(launch_theta + .1_deg, launch_vel);
                error_0 = error;
                // Gradient descent loop - exits if it gets stuck at depth 50
                while(error > 1._in && depth < 50) {
                    // Once error starts increasing, high r is leading to overshoot and oscillation
                    if(error > error_0) oscillating = true;
                    // Once oscillation begins, have r approach 2 - smaller value better at high curvature
                    if(oscillating == true) r = .5*r + .2;
                    // Updates launch parameters using the gradient
                    launch_vel = launch_vel - r*(error_dv - error).convert(okapi::inch)/(.01)*1._in/1._s;
                    launch_theta = launch_theta - r*(error_dt - error).convert(okapi::inch)/(.1)*1._deg;
                    error_0 = error;
                    error = turret.trajectory_error(launch_theta, launch_vel);
                    error_dv = turret.trajectory_error(launch_theta, launch_vel + .01*1._in/1._s);
                    error_dt = turret.trajectory_error(launch_theta + .1_deg, launch_vel);
                    depth++;
                }
                // Outputs failure message if it fails to find acceptable launch parameters
                if(depth == 50 && error > 1_in) {
                    std::cout << "Unable to compute trajectory :(" << std::endl;
                    std::cout << "error: " << error.convert(okapi::inch) << std::endl;
                    std::cout << "mps: " << launch_vel.convert(okapi::mps) << std::endl;
                    std::cout << "theta: " << launch_theta.convert(okapi::degree) << std::endl;
                }
            }
            target_mutex.give();
            // Converts the output theta into a value that prevents overrotation and only cares about equivalent angle, not absolute angle
            // Launch theta negated to convert to clockwise equivalent for compatibility with okapi odometry
            turret.set_target_angle(unnormalized_rotation_to(launch_theta - turret.orientation - chassis.dangledt * .5_s, turret.turret_absolute_angle)); 
            turret.set_target_velocity(launch_vel);
            pros::delay(50);
        } else if(control_phase == "opcontrol") {
            if(!turret.turret_controller -> isDisabled()) turret.turret_controller -> flipDisable();
            turret.turret_mtr.moveVoltage(12000 * partner.getAnalog(okapi::ControllerAnalog::leftX));
        }
        pros::delay(20);
    }
}

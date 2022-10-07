#include "main.h"

double trajectory_error(double x_pos, double y_pos, double x_vel, double y_vel, double theta, double launch_vel) {
  // Calculates the distance in a given trajectory from the ideal end point
  //
  //       [|v_0|cos(phi)cos(theta)]
  // v_0 = [|v_0|cos(phi)sin(theta)]
  //       [     |v_0|sin(phi)     ]
  //
  // h_t = h_launcher + |v_0|sin(phi)t + g/2*t^2
  // 0 = (h_launcher - h_goal) + |v_0|sin(phi)t + g/2*t^2
  // Then use the quadratic formula...
  double discriminant = pow((launch_vel*sin(launch_angle)),2) - 4*(launch_height - goal_height)*(g/2);
  if(discriminant < 0) { // Complex root -> doesn't reach height
    std::cout << "Trajectory with theta " << theta << " and launch velocity" << launch_vel << " in/s will never reach target height." << std::endl;
    return 9001;
  }
  double t = (-launch_vel*sin(launch_angle) - sqrt(discriminant))/g;
  double x_end_pos = (x_pos + offset*cos(theta + alpha)) + t*launch_vel*cos(launch_angle)*cos(theta) + t*x_vel;
  double y_end_pos = (y_pos + offset*sin(theta + alpha)) + t*launch_vel*cos(launch_angle)*sin(theta) + t*y_vel;
  double error = sqrt(pow(goal_x-x_end_pos,2) + pow(goal_y-y_end_pos,2));
  return error;
}

double ideal_velocity(double x_pos, double y_pos, double target_x, double target_y, double target_height) {
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
  double dist = sqrt(pow((target_x-x_pos),2) + pow((target_y-y_pos),2));
  double root = g*pow(dist,2)/((2*pow(cos(launch_angle),2))*(target_height - launch_height - dist*tan(launch_angle)));
  if(root <= 0) { // Complex root -> infinite velocity+ to hit target
    std::cout << "Too close to goal to fire" << std::endl;
    return 9001;
  }
  double launch_vel = sqrt(root);
  return launch_vel;
}

double angle_to_goal(double x_pos, double y_pos, double target_x, double target_y) {
  // Calculates the angle from the robot to the goal
  double theta = atan2(target_y - y_pos, target_x - x_pos);
  return theta;
}

void auto_aim() {
  // When enabled, automatically adjusts turret position and flywheel speed to target the goal
  double x_pos;
  double y_pos;
  double x_vel;
  double y_vel;
  double launch_theta;
  double launch_vel;
  double error_0;
  double error;
  double error_dv;
  double error_dt;
  int depth;
  bool oscillating;
  double r;
  while(auto_aim_enabled == true) {
    std::cout << "Beginning Targeting Calculations..." << std::endl;
    x_pos = chassis_controller -> getState().x.convert(okapi::inch);
    y_pos = chassis_controller -> getState().y.convert(okapi::inch);
    if(aiming_for_low_goal) {
      // Assumes ideal values are true when aiming for the low goal, since accuracy is less important
      launch_theta = angle_to_goal(x_pos, y_pos, low_goal_x, low_goal_y);
      launch_vel = ideal_velocity(x_pos, y_pos, low_goal_x, low_goal_y, 0);
    } else {
      // Determines best launch parameters for high goal using gradient descent
      // Uses innacurate estimation as starting point
      launch_theta = angle_to_goal(x_pos, y_pos, goal_x, goal_y);
      launch_vel = ideal_velocity(x_pos, y_pos, goal_x, goal_y, goal_height);
      // Lock in robot velocity to avoid errors since this is multithreaded
      x_vel = global_x_vel;
      y_vel = global_y_vel;
      error = 9000;
      error_0 = 9000;
      depth = 0;
      // Initial coefficient for gradient descent - good for low curvature
      r = 10;
      oscillating = false;
      // Calculate initial error outside loop to remove need to store past launch parameters
      error = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta, launch_vel);
      error_dv = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta, launch_vel + .001);
      error_dt = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta + .001, launch_vel);
      error_0 = error;
      // Gradient descent loop - exits if it gets stuck at depth 50
      while(error > 2 && depth < 50) {
        // Once error starts increasing, high r is leading to overshoot and oscillation
        if(error > error_0) oscillating = true;
        // Once oscillation begins, have r approach .2 - smaller value better at high curvature
        if(oscillating == true) r = .5*r + .1;
        // Updates launch parameters using the gradient
        launch_vel = launch_vel - r*(error_dv - error)/.001;
        launch_theta = launch_theta - r*(error_dt - error)/.001;
        error_0 = error;
        error = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta, launch_vel);
        error_dv = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta, launch_vel + .001);
        error_dt = trajectory_error(x_pos, y_pos, x_vel, y_vel, launch_theta + .001, launch_vel);
        depth++;
      }
    }
    std::cout << "Targeting Complete" << std::endl;
    std::cout << "Trajectory Theta: " << launch_theta << std::endl;
    std::cout << "Trajectory Velocity: " << launch_vel << std::endl;
    turret_controller -> setTarget(launch_theta/(2*PI)*360 - inertial.get_yaw()); // Convert from radians to degrees
    flywheel_controller -> setTarget(launch_vel/(3*PI)*2);
  }
}

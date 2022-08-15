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
    std::cout << "Trajectory with theta " << theta << " and launch velocity" << launch_vel << " in/s^2 will never reach target height." << std::endl;
    return 9001;
  }
  double t = (-launch_vel*sin(launch_angle) + sqrt(discriminant))/(2*(launch_height-goal_height));
  double x_end_pos = x_pos + t*launch_vel*cos(launch_angle)*cos(theta) + t*x_vel;
  double y_end_pos = y_pos + t*launch_vel*cos(launch_angle)*sin(theta) + t*y_vel;
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
  double root = g*pow(dist,2)/((2*pow(cos(launch_angle),2))*(target_height-launch_height-dist*tan(launch_angle)));
  if(root <= 0) { // Complex root -> infinite velocity+ to hit target
    std::cout << "Too close to goal to fire" << std::endl;
    return 9001;
  }
  double launch_vel = sqrt(root);
  return launch_vel;
}

double angle_to_goal(double x_pos, double y_pos, double target_x, double target_y) {
  // Calculates the angle from the robot to the goal
  double theta = atan2(target_y-y_pos,target_x-x_pos);
  if(theta < 0) {
    theta = 2*PI + theta;
  }
  return theta;
}

void auto_aim() {
  // When enabled, automatically adjusts turret position and flywheel speed to target the goal
  double x_pos;
  double y_pos;
  double x_vel;
  double y_vel;
  double current_error;
  double lowest_error;
  double best_theta;
  double best_vel;
  double new_best_theta;
  double new_best_vel;
  double vel_increment;
  double theta_increment;
  double error;
  int depth;
  while(auto_aim_enabled == true) {
    std::cout << "Beginning Targeting Calculations..." << std::endl;
    x_pos = chassis_controller -> getState().x.convert(okapi::inch);
    y_pos = chassis_controller -> getState().y.convert(okapi::inch);
    if(aiming_for_low_goal) {
      // Assumes ideal values are true when aiming for the low goal, since accuracy is less impaortant
      best_theta = angle_to_goal(x_pos, y_pos, low_goal_x, low_goal_y);
      best_vel = ideal_velocity(x_pos, y_pos, low_goal_x, low_goal_y, 0);
    } else {
      best_theta = angle_to_goal(x_pos, y_pos, goal_x, goal_y);
      best_vel = ideal_velocity(x_pos, y_pos, goal_x, goal_y, goal_height);
      x_vel = global_x_vel;
      y_vel = global_y_vel;
      vel_increment = sqrt(pow(x_vel,2.)+pow(y_vel,2.))/5.;
      theta_increment = PI/15.;
      lowest_error = 1000;
      depth = 0;
      while(lowest_error > 2) {
        // Looks through a 10x10 grid of potential launch velocities and angles, then repeatedly zooms in on the closest launch specifications until desired accuracy is reached
        // This assumes that it's impossible for the best launch parameters to be closer to a less accurate set of more zoomed out parameters than the best zoomed out parameters
        // If this becomes an issue or computation time isn't significant, I can fix this by increasing the increments to include points around the next closest parameter sets
        for(int vel_i = -5; vel_i<5; vel_i++) {
          for(int theta_i = -5; theta_i<5; theta_i++) {
            error = trajectory_error(x_pos, y_pos, x_vel, y_vel, best_theta + theta_i * theta_increment, best_vel + vel_i * vel_increment);
            if(error < lowest_error) {
              lowest_error = error;
              new_best_theta = best_theta + theta_i * theta_increment;
              new_best_vel = best_vel + vel_i * vel_increment;
            }
          }
        }
        best_theta = new_best_theta;
        best_vel = new_best_vel;
        vel_increment = vel_increment/5;
        theta_increment = theta_increment/5;
        depth++;
        // Terminates prematurely if calculations get stuck. Calibration needed to find appropriate depth
        if(depth == 5) {
          break;
        }
      }
    }
    std::cout << "Targeting Complete" << std::endl;
    std::cout << "Trajectory Theta: " << best_theta << std::endl;
    std::cout << "Trajectory Velocity: " << best_vel << std::endl;
    turret_controller -> setTarget(best_theta/(2*PI)*360 - inertial.get_yaw()); // Convert from radians to degrees
    flywheel_controller -> setTarget(best_vel/(3.25*PI)*2);
  }
}

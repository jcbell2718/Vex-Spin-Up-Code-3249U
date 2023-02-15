#include "global_variables.hpp"
#include "main.h"

void display_on_both_controllers(std::string text, int line, int col) {
    master.setText(line, col, text + "                ");
    pros::delay(120);
    partner.setText(line, col, text + "                ");
    pros::delay(120);
}

void display_on(okapi::Controller controller, std::string text, int line, int col) {
    controller.setText(line, col, text + "                ");
    pros::delay(120);
}

void clear_controllers() {
    master.clear();
    pros::delay(120);
    partner.clear();
    pros::delay(120);
}

void clear_controller(okapi::Controller controller) {
    controller.clear();
    pros::delay(120);
}

void clear_line_on(okapi::Controller controller, int line) {
    controller.clearLine(line);
    pros::delay(120);
}

void clear_line_on_both_controllers(int line) {
    master.clearLine(line);
    pros::delay(120);
    partner.clearLine(line);
    pros::delay(120);
}

void controller_lcd_fn() {
    Chassis chassis = *chassis_pointer;
    Intake intake = *intake_pointer;
    Turret turret = *turret_pointer;
    while(true) {
        if(control_phase == "opcontrol") {
            if(turret.auto_aim_enabled) {
                display_on_both_controllers("Auto-Aim: ON", 0);
                if(turret.aiming_for_low_goal) display_on_both_controllers("Target: Low", 1);
                else display_on_both_controllers("Target: High", 1);
            } else {
                display_on_both_controllers("Auto-Aim: OFF", 0);
                display_on_both_controllers("RPM: " + std::to_string(turret.launch_RPM), 1);
            }
            if(chassis.front_left_mtr.isOverTemp() || chassis.front_right_mtr.isOverTemp() || chassis.back_left_mtr.isOverTemp() || chassis.back_right_mtr.isOverTemp() || turret.turret_mtr.isOverTemp() || intake.intake_mtr.isOverTemp() || turret.flywheel_mtr_1.isOverTemp() || turret.flywheel_mtr_2.isOverTemp()) {
                display_on_both_controllers("Over Temp!", 2);
            } else clear_line_on_both_controllers(2);
        } else if(control_phase == "initialize") {
            // Disables auto-aim and the intake so they don't run during setup selection
            turret.auto_aim_enabled = false;
            intake.intake_mode = false;

            // Auton selection menu through the controller
            clear_controllers();
            display_on(master, "Selecting Auton:", 0);
            display_on(master, auton, 1);
            bool selection_made = false;
            while(!selection_made) {
                if(master_A.changedToPressed()) {
                    // Submit choice
                    selection_made = true;
                } else if(master_left.changedToPressed()) {
                    // Cycle through choices backwards
                    auton_index--;
                    if(auton_index < 0) auton_index = auton_list.size() - 1;
                    auton = auton_list[auton_index];
                    display_on(master, auton, 1);
                } else if(master_right.changedToPressed()) {
                    // Cycle through choices forwards
                    auton_index++;
                    if(auton_index == auton_list.size()) auton_index = 0;
                    auton = auton_list[auton_index];
                    display_on(master, auton, 1);
                }
                pros::delay(20);
            }

            // Alliance selection menu through the controller
            selection_made = false;
            display_on(master, "Selecting Alliance:", 0);
            display_on(master, intake.alliance_color, 1);
            while(!selection_made) {
                if(master_A.changedToPressed()) {
                    // Submit choice
                    selection_made = true;
                } else if(master_left.changedToPressed() || master_right.changedToPressed()) {
                    // Cycle through choices
                    if(intake.alliance_color == "red") intake.alliance_color = "blue";
                    else intake.alliance_color = "red";
                    display_on(master, intake.alliance_color, 1);
                }
                pros::delay(20);
            }

            auton_setup();

            // Display final configuration
            display_on(master, auton, 0);
            display_on(master, "  -Setup Loaded-  ", 2);

            // Auton or opcontrol can be enabled without competition switch
            // Also holds to prevent automatic opcontrol when wanting to plug into field control
            while(!pros::competition::is_connected()) {
                if(master_Y.changedToPressed()) {
                    control_phase = "autonomous";
                    autonomous();
                    break;
                } else if(master_A.changedToPressed()) {
                    control_phase = "opcontrol";
                    break;
                } else if(master_B.changedToPressed()) {
                    control_phase = "initialize";
                    break;
                }
            }
        } else if(control_phase == "disabled") {
            clear_line_on_both_controllers(0);
            display_on_both_controllers("disabled lmao", 1);
            clear_line_on_both_controllers(2);
        } else {
            pros::delay(200);
        }
    }
}
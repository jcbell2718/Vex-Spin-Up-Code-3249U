#include "main.h"

void display_on_both_controllers(std::string text, int line, int col) {
    master.setText(line, col, text + "                ");
    pros::delay(120);
    partner.setText(line, col, text + "                ");
    pros::delay(120);
}

void clear_controllers() {
    master.clear();
    pros::delay(120);
    partner.clear();
    pros::delay(120);
}

void display_on(okapi::Controller controller, std::string text, int line, int col) {
    controller.setText(line, col, text + "                ");
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
            } else display_on_both_controllers("                ", 2);
        } else {
            pros::delay(200);
        }
    }
}
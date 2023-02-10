#include "console_output.hpp"
#include "controller_lcd.hpp"
#include "main.h"
#include "global_variables.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "pros/misc.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    std::cout << "did ya get this far?";
    Chassis chassis;
    Turret turret;
    Intake intake;
    chassis.reset_encoders();
    chassis_pointer = &chassis;
    turret_pointer = &turret;
    intake_pointer = &intake;
	
	// Grafana
	set_up_grafana();

	// Initialize tasks
	pros::Task auto_aiming(auto_aim);
	pros::Task intake_control(intake_regulation_fn);
    pros::Task velocity_recording(velocity_recording_fn);
    pros::Task console_output(console_output_fn);
    pros::Task controller_lcd(controller_lcd_fn);

	// Setup selection is in loop so autons can be run repeatedly without turning off the program
	// DON'T TURN ON THE ROBOT PLUGGED INTO FIELD CONTROL
	while(!pros::competition::is_connected()) {
        control_phase = "initialization";

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
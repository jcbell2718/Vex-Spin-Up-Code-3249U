#include "main.h"
#include "pros/misc.h"

void intake_regulation_function(void) {
    // Regulates the intake and indexer during both autonomous and driver control periods
    while(true) {
        if(intake_enabled) {
            if((auto_aim_enabled && disk_switch.changedToPressed()) || (!auto_aim_enabled && partner_L2.changedToPressed() && !pros::competition::is_autonomous())) {
                indexing = true;
                indexer.set_value(true);
                intake_timer.placeMark();
            }
            if(intake_timer.getDtFromMark().convert(okapi::millisecond) > 300) indexing = false;
            else if(intake_timer.getDtFromMark().convert(okapi::millisecond) > 150) indexer.set_value(false);
            if(indexing) intake_mtr.moveVoltage(0);
            else intake_mtr.moveVoltage(12000);
        } else {
            intake_mtr.moveVoltage(0);
        }
        pros::delay(20);
    }
}
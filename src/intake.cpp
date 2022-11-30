#include "global_variables.hpp"
#include "main.h"
#include "pros/misc.h"

void intake_regulation_function() {
    // Regulates the intake, indexer, and roller mechanism during both autonomous and driver control periods
    okapi::Timer intake_timer;
    intake_timer.placeMark();
    pros::delay(600);
    while(true) {
        if(intake_enabled) {
            if((auto_aim_enabled && disk_switch.changedToPressed()) || (!auto_aim_enabled && partner_up.changedToPressed() && !partner_B.isPressed())) {
                indexing = true;
                intake_timer.placeMark();
            }
            if(intake_timer.getDtFromMark().convert(okapi::millisecond) > 600) indexing = false;
            else if(intake_timer.getDtFromMark().convert(okapi::millisecond) > 300) indexer.set_value(false);
            else if(intake_timer.getDtFromMark().convert(okapi::millisecond) > 100) indexer.set_value(true);
            if(indexing) intake_mtr.moveVoltage(0);
            else intake_mtr.moveVoltage(12000);
        } else if(partner_up.changedToPressed() && !partner_B.isPressed())  turn_roller(alliance_color);
        else intake_mtr.moveVoltage(0);
        pros::delay(20);
    }
}

bool turn_roller(std::string target_color) {
    // Turns the roller so that the target color is up
    // Rotates until the opposing color is up, then rotates back until the target color is to guarantee the center is the target color
    // Assumes optical sensor is facing bottom half of roller, so the opposite side of that detected is on top
    // Exits after 3 seconds in case the roller isn't actually contacted, returns true if it succeeds and false otherwise
    okapi::Timer roller_timer;
    roller_timer.placeMark();
    if(target_color == "red") {
        while(roller_optical.get_rgb().blue > 150 && roller_timer.getDtFromMark() < 3_s) intake_mtr.moveVoltage(12000);
        while(roller_optical.get_rgb().red > 150 && roller_timer.getDtFromMark() < 3_s) intake_mtr.moveVoltage(-12000);
    } else {
        while(roller_optical.get_rgb().red > 150 && roller_timer.getDtFromMark() < 3_s) intake_mtr.moveVoltage(12000);
        while(roller_optical.get_rgb().blue > 150 && roller_timer.getDtFromMark() < 3_s) intake_mtr.moveVoltage(-12000);
    }
    return roller_timer.getDtFromMark() < 3_s;
}
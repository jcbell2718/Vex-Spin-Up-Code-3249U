#include "main.h"

Intake::Intake() :
    intake_mtr(INTAKE_MOTOR_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations),
    disk_switch(LIMIT_SWITCH_PORT),
    indexer(INDEXER_PORT),
    PTO(PTO_PORT),
    roller_optical(ROLLER_OPTICAL_PORT),
    alliance_color("blue")
{
    intake_mode = false;
    indexing = false;
    roller_optical.set_led_pwm(100);
}

bool Intake::turn_roller() {
    // Turns the roller so that the target color is up
    // Rotates until the opposing color is up, then rotates back until the target color is to guarantee the center is the target color
    // Assumes optical sensor is facing bottom half of roller, so the opposite side of that detected is on top
    // Exits after 3 seconds in case the roller isn't actually contacted, returns true if it succeeds and false otherwise
    okapi::Timer roller_timer;
    roller_timer.placeMark();
    if(alliance_color == "red") {
        // on Blue
        while(roller_optical.get_hue() > 140 && roller_timer.getDtFromMark() < 3_s) {
            intake_mtr.moveVoltage(8000);
            pros::delay(5);
        }
        // Red
        while(roller_optical.get_hue() < 140 && roller_timer.getDtFromMark() < 3_s) {
            intake_mtr.moveVoltage(-8000);
            pros::delay(5);
        }
        intake_mtr.moveVoltage(0);
        intake_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    } else {
        while(roller_optical.get_hue() < 140 && roller_timer.getDtFromMark() < 3_s) {
            intake_mtr.moveVoltage(8000);
            pros::delay(5);
        }
        while(roller_optical.get_hue() > 140 && roller_timer.getDtFromMark() < 3_s) {
            intake_mtr.moveVoltage(-8000);
            pros::delay(5);
        }
        intake_mtr.moveVoltage(0);
        intake_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    }
    pros::delay(200);
    intake_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    return roller_timer.getDtFromMark() < 3_s;
}

void Intake::index() {
    if(!indexing) {
        indexing = true;
        intake_mtr.moveVoltage(0);
        pros::delay(100);
        indexer.set_value(true);
        pros::delay(400);
        indexer.set_value(false);
        pros::delay(300);
        indexing = false;
    }
}

void Intake::PTO_to_intake() {
    PTO.set_value(false);
	intake_mode = true;
    intake_mtr.moveVoltage(12000);
    pros::delay(500);
    intake_mtr.moveVoltage(0);
}

void Intake::PTO_to_roller_mech() {
    PTO.set_value(true);
	intake_mode = false;
    intake_mtr.moveVoltage(12000);
    pros::delay(500);
    intake_mtr.moveVoltage(0);
}

Intake intake = Intake();

void intake_regulation_fn() {
    // Regulates the intake, indexer, and roller mechanism during both autonomous and driver control periods
    while(true) {
        // Intake PTO
		if(partner_left.changedToPressed()) intake.PTO_to_roller_mech();
		else if(partner_right.changedToReleased()) intake.PTO_to_intake();
        if(intake.intake_mode) {
            if((turret.auto_aim_enabled && intake.disk_switch.changedToPressed()) || (partner_up.changedToPressed())) intake.index();
            intake.intake_mtr.moveVoltage(12000);
        } else {
            if((turret.auto_aim_enabled && intake.disk_switch.changedToPressed()) || (!turret.auto_aim_enabled && partner_up.changedToPressed())) intake.index();
            if(partner_A.changedToPressed())  intake.turn_roller();
            intake.intake_mtr.moveVoltage(0);
        }
        pros::delay(20);
    }
}
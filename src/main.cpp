#include "main.h"
#include "util.hpp"
#include "auton.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::IMU inertial(10);

PID turnPID(2.5, 0.0001, 1);
PID drivePID(10, 0.00, 3);
PID liftPID(0, 0, 0);
PID clawPivotPID(0, 0, 0);

void printStats(void*)
{
    while (true)
    {
        pros::lcd::print(4, "X: %f Y: %f", odomX, odomY);
        pros::lcd::print(5, "Theta %f", getRotation(inertial.get_heading()));
        pros::delay(200);
    }
}

pros::Task* updateEncoderTask;
pros::Task* outputTask;

void initialize()
{
    
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
    inertial.reset();

    left_mg.tare_position_all();
    right_mg.tare_position_all();

    while (inertial.is_calibrating()) {
        pros::delay(10);
    }

	pros::Task updateEncoderTask(updateCoordinatesWithEncoders);
    pros::Task outputTask(printStats);
}


void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{

}


void autonomous()
{
    driveToPoint(24, 72, false);
    driveClamp = 40;
    driveToPoint(72, 120, false);
    turnToAngle(-45);
    driveToPoint(96, 72, false);
    turnToAngle(-135);
    driveToPoint(72, 48, false);
    turnToAngle(135);
    driveToPoint(24, 72, false);
}


void opcontrol() {
	while (true) {
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		left_mg.move(dir - turn);
		right_mg.move(dir + turn);
		pros::delay(20);

        // // 2. Read the total current draw (mA) for both sides
        // // get_current_draws() returns a vector of currents for each motor
        // auto left_currents = left_mg.get_current_draw_all();
        // auto right_currents = right_mg.get_current_draw_all();

        // int32_t left_total = 0;
        // int32_t right_total = 0;

        // for(int m : left_currents)  left_total += m;
        // for(int m : right_currents) right_total += m;

        // // 3. Print the diagnostic data to the screen
        // pros::lcd::print(1, "Left Total: %d mA", left_total);
        // pros::lcd::print(2, "Right Total: %d mA", right_total);
        
        // // Quick visual reference to check for side-to-side balance
        // int32_t difference = std::abs(left_total - right_total);
        // pros::lcd::print(3, "Diff: %d mA", difference);
	}
}
#include "main.h"
#include "util.hpp"
#include "auton.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-10, 16, -17});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({11, -2, 9});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::IMU inertial(13);
pros::Rotation strafingWheel(18);
pros::Rotation forwardWheel(15);
pros::Rotation lift(12);

pros::MotorGroup lift_mg({19, -20});
pros::MotorGroup claw_mg({14,-3});
pros::ADIDigitalOut claw_piston('a');

PID turnPID(2, 0.000, 0.625);
PID drivePID(10, 0.00, 0.75);
PID liftPID(11, 0, 2.5);
PID clawPivotPID(0, 0, 0);

double odomX = 13;
double odomY = 132;

void printStats(void*)
{
    while (true)
    {
        pros::lcd::print(4, "X: %f Y: %f", odomX, odomY);
        pros::lcd::print(5, "Theta: %f", getRotation(inertial.get_heading()));
        pros::lcd::print(6, "Lift Angle: %f", getRotation(lift.get_position()/100));

        master.print(1, 1, "Left MG: %f", left_mg.get_temperature_all());
        master.print(2, 1, "Right MG: %f", right_mg.get_temperature_all());
        pros::delay(200);
    }
}

pros::Task* updateEncoderTask;
pros::Task* outputTask;

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Initalizing...");
    master.set_text(1, 1, "Initializing...");
    inertial.reset();

    left_mg.tare_position_all();
    right_mg.tare_position_all();
    strafingWheel.reset_position();
    forwardWheel.reset_position();
    lift_mg.tare_position_all();
    lift.reset_position();

    lift_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);

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

void autonomousRight()
{
    driveClamp = 127;
    turnClamp = 100;
    liftClamp = 127;
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Move forward with preload.
    claw_mg.move(-127);
    claw_piston.set_value(false);
    driveToPoint(17, 132, false);

    // Turn towards goal. Prepare lift.
    turnToAngle(-55);
    turnClamp = 0;
    driveBlindWithAngle(127, -55, 500);
    driveBlindWithAngle(-127, -55, 250);
    turnClamp = 100;
    pros::Task lift_up_first([](void* param) -> void {
        liftToAngle(-23);
    }, nullptr, "Lift 1");
    turnToAngle(-85);
    driveClamp = 100;

    // Drive to goal. Release claw.
    turnClamp = 15;
    driveToPoint(25, 98, false, 1500);
    lift_mg.move(25);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(500);
    turnClamp = 100;
    claw_piston.set_value(true);
    claw_mg.move(0);
    lift_mg.move(0);

    // Go back to loader.
    driveClamp = 127;
    driveBlindWithAngle(-127, -90, 200);
    pros::Task lift_down([](void* param) -> void {
        liftToAngle(0);
    }, nullptr, "Lift 2");
    turnToAngle(90);
    driveToPoint(24, 129, false);

    // Retrieve new cup and pin.
    turnToAngle(170);
    claw_piston.set_value(true);
    claw_mg.move(-127);
    driveBlindWithAngle(27, 170, 2000);
    claw_piston.set_value(false);

    // Escape back to goal.
    driveBlindWithAngle(-40, 170, 400);
    turnToAngle(-90);
    pros::Task lift_up_second([](void* param) -> void {
        liftToAngle(-45);
    }, nullptr, "Lift 3");
    driveClamp = 100;
    driveToPoint(24, 108, false, 1250);
    left_mg.move(45);
    right_mg.move(45);
    lift_mg.move(25);
    pros::delay(650);

    // Score and go backwards.
    claw_piston.set_value(true);
    claw_mg.move(20);
    lift_mg.move(0);
    driveBlindWithAngle(-127, -90, 250);
    
    claw_mg.move(0);
    pros::Task lift_down_second([](void* param) -> void {
        liftToAngle(0);
    }, nullptr, "Lift 4");
    turnToAngle(90);
    driveToPoint(24, 135, false);

    // Retrieve new cup and pin.
    turnToAngle(170);
    claw_piston.set_value(true);
    claw_mg.move(-127);
    driveBlindWithAngle(25, 170, 1500);
    claw_piston.set_value(false);

    lift_mg.move(0);
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void autonomousLeft()
{
    driveClamp = 127;
    turnClamp = 100;
    liftClamp = 127;
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Move forward with preload.
    claw_mg.move(-127);
    claw_piston.set_value(false);
    driveToPoint(17, 12, false);

    // Turn towards goal. Prepare lift.
    turnToAngle(55);
    turnClamp = 0;
    driveBlindWithAngle(127, 55, 500);
    driveBlindWithAngle(-127, 55, 250);
    turnClamp = 100;
    pros::Task lift_up_first([](void* param) -> void {
        liftToAngle(-23);
    }, nullptr, "Lift 1");
    turnToAngle(85);
    driveClamp = 100;

    // Drive to goal. Release claw.
    turnClamp = 15;
    driveToPoint(25, 40, false, 1500);
    lift_mg.move(25);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(500);
    turnClamp = 100;
    claw_piston.set_value(true);
    claw_mg.move(0);
    lift_mg.move(0);

    // Go back to loader.
    driveClamp = 127;
    driveBlindWithAngle(-127, 90, 200);
    pros::Task lift_down([](void* param) -> void {
        liftToAngle(0);
    }, nullptr, "Lift 2");
    turnToAngle(-90);
    driveToPoint(24, 14, false);

    // Retrieve new cup and pin.
    turnToAngle(-170);
    claw_piston.set_value(true);
    claw_mg.move(-127);
    driveBlindWithAngle(25, -170, 1500);
    claw_piston.set_value(false);

    // Escape back to goal.
    driveBlindWithAngle(-40, -170, 400);
    turnToAngle(90);
    pros::Task lift_up_second([](void* param) -> void {
        liftToAngle(-35);
    }, nullptr, "Lift 3");
    driveClamp = 100;
    driveToPoint(24, 41, false, 1500);
    left_mg.move(45);
    right_mg.move(45);
    lift_mg.move(25);
    pros::delay(400);

    // Score and go backwards.
    claw_piston.set_value(true);
    claw_mg.move(20);
    lift_mg.move(0);
    driveBlindWithAngle(-127, 90, 250);
    
    claw_mg.move(0);
    pros::Task lift_down_second([](void* param) -> void {
        liftToAngle(0, 3000);
    }, nullptr, "Lift 4");
    turnToAngle(-90);
    driveToPoint(24, 11, false);

    // Retrieve new cup and pin.
    turnToAngle(-170);
    claw_piston.set_value(true);
    claw_mg.move(-127);
    driveBlindWithAngle(25, -170, 1500);
    claw_piston.set_value(false);

    lift_mg.move(0);
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void autonomous()
{
    autonomousRight();
}

void opcontrol() {
	while (true) {
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		left_mg.move(dir + turn);
		right_mg.move(dir - turn);

        if (master.get_digital(DIGITAL_R2)) {
            lifting = true;
            lift_mg.move(127);
        }
        else if(master.get_digital(DIGITAL_R1)) {
            lifting = true;
            lift_mg.move(-127);
        }
        else {
            lift_mg.move(0);
        }

        if (master.get_digital(DIGITAL_L2)) {
            claw_mg.move(50);
        }
        else if(master.get_digital(DIGITAL_L1)) {
            claw_mg.move(-127);
        }
        else {
            claw_mg.move(0);
        }

        if (master.get_digital_new_press(DIGITAL_RIGHT)) {
            claw_piston.set_value(true);
        }
        if (master.get_digital_new_press(DIGITAL_DOWN)) {
            claw_piston.set_value(false);
        }

		pros::delay(20);
	}
}
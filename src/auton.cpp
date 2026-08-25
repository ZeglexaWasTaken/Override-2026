#include "main.h"
#include "util.hpp"

double driveClamp = 127;
double turnClamp = 127;
double liftClamp = 127;

bool clawFaceToggle = true; // True for forwards, False for backwards

double odomX = 8.75;
double odomY = 72;

pros::Distance distanceLeft(1);
pros::Distance distanceRight(2);
pros::Distance distanceForward(3);


pros::Rotation lift(7);
pros::Motor claw_pivot_joint(8);
pros::MotorGroup lift_mg({9, 10}, pros::MotorGears::red);

double coordinateCorrection(pros::Distance sensor, bool yAxis)
{
    double trueDistance = mmToInches(sensor.get_distance());


    return trueDistance;
}

// Takes an angle in degrees and coverts it to a range within (-180,180). MUST USE TO CONVERT IMU OUTPUTS SINCE THEY'RE WEIRD.
// I know it's coded poorly but it works.
double getRotation(double unboundedRot)
{
    unboundedRot -= 360*floor(unboundedRot / 360);

    if (abs(unboundedRot) > 180)
    {
        unboundedRot = ((unboundedRot / -abs(unboundedRot))*180) + ((abs(unboundedRot) - 180) * (unboundedRot / abs(unboundedRot)));
    }

    return unboundedRot;
}

// Turn to a raw angle in degrees. ONLY ENTER ANGLES WITHIN A (-180,180) RANGE.
void turnToAngle(double targetAngle, int timeout = 999999999)
{
    turnPID.reset();
    int t = 0;
    double currentAngle = getRotation(inertial.get_rotation());

    while (abs(targetAngle - currentAngle) > 0.01)
    {
        t += 20;

        currentAngle = getRotation(inertial.get_rotation());
        double power = turnPID.compute(targetAngle, currentAngle);
        power = std::clamp(power, -turnClamp, turnClamp);

        left_mg.move(-power);
        right_mg.move(power);

        if (t >= timeout)
        {
            break;
        }
        pros::delay(20);
    }

    left_mg.move(0);
    right_mg.move(0);
}

// Turn towards point X and Y. Good for sharp turns.
void turnToPoint(double targetX, double targetY, bool reverse, int timeout = 999999999)
{
    turnPID.reset();
    int t = 0;

    double targetAngle = radToDegrees(atan2(targetY - odomY, targetX - odomX));
    targetAngle = getRotation(targetAngle);

    if (reverse)
    { 
        // Since we are turning backwards, the robot needs to rotate 180 from the forward target angle in order to be facing in reverse.
        if (targetAngle >= 0)
        {
            targetAngle = getRotation(targetAngle + 180);
        }
        else
        {
            targetAngle = getRotation(targetAngle - 180);
        }
    }

    double currentAngle = getRotation(inertial.get_rotation());

    while (abs(targetAngle - currentAngle) > 1)
    {
        t += 20;

        currentAngle = getRotation(inertial.get_rotation());
        double power = turnPID.compute(targetAngle, currentAngle);
        power = std::clamp(power, -turnClamp, turnClamp);

        left_mg.move(-power);
        right_mg.move(power);

        if (t >= timeout)
        {
            break;
        }
        pros::delay(20);
    }

    left_mg.move(0);
    right_mg.move(0);
}

// IMPORTANT: THIS SHOULD ALWAYS BE RUNNING IN A TASK
// Updates the odomX and odomY variables via drive encoders for use in auton driving commands. 
// Note that this style of coordinate finding is unreliable and will cause drift. Use with distance distance sensors for best results.
void updateCoordinatesWithEncoders()
{
    double prevLeft = 0;
    double prevRight = 0;

    while (true)
    {
        double leftDelta = left_mg.get_position() - prevLeft;
        double rightDelta = right_mg.get_position() - prevRight;

        double forwardDelta = degToInches((leftDelta + rightDelta) / 2, wheelCircumference, gearRatio);
        double theta = degToRadians(getRotation(inertial.get_rotation()));

        prevLeft = left_mg.get_position();
        prevRight = right_mg.get_position();

        double odomXDelta = forwardDelta * cos(theta);
        double odomYDelta = forwardDelta * sin(theta);

        odomY += odomYDelta;
        odomX += odomXDelta;

        
        pros::delay(10);
    }
}

// Drive to point X and Y. Angle is determined automatically to get to destination. Angle after command is finished NOT consistent.
// Do not use this command if there are objects in the way or if turning is too sharp. Use turnToPoint for sharp turning, THEN drive to point.
void driveToPoint(double targetX, double targetY, bool reverse, int timeout = 999999999)
{
    drivePID.reset();
    turnPID.reset();
    int t = 0;
    // double initialDistance = abs(sqrt(pow(targetX - odomX, 2) + pow(targetY - odomY, 2)));

    while (abs(sqrt(pow(targetX - odomX, 2) + pow(targetY - odomY, 2))) > 0.001)
    {
        t += 20;

        double currentDistance = abs(sqrt(pow(targetX - odomX, 2) + pow(targetY - odomY, 2)));
        double drivePower = drivePID.compute(0, -currentDistance);
        drivePower = std::clamp(drivePower, -driveClamp, driveClamp);

        double targetAngle = radToDegrees(atan2(targetY - odomY, targetX - odomX));
        targetAngle = getRotation(targetAngle);

        if (reverse)
        { 
            // Since we are driving backwards, the robot needs to rotate 180 from the forward target angle in order to be facing in reverse.
            if (targetAngle >= 0)
            {
                targetAngle = getRotation(targetAngle + 180);
            }
            else
            {
                targetAngle = getRotation(targetAngle - 180);
            }
        }

        double turnPower = turnPID.compute(targetAngle, getRotation(inertial.get_rotation()));
        turnPower = std::clamp(turnPower, -turnClamp, turnClamp);

        // Switch operator signs if reverse is true.
        left_mg.move((drivePower * -boolToSign(reverse)) - (turnPower));
        right_mg.move((drivePower * -boolToSign(reverse)) + (turnPower));

        if (t >= timeout)
        {
            break;
        }
        pros::delay(20);
    }
    
    left_mg.move(0);
    right_mg.move(0);
}

// // IMPORTANT: THIS SHOULD ALWAYS BE RUNNING IN A WHILE LOOP IN A TASK
// // This code manages the claw's pitch pivot joint both during autonomous and in driver control.
// void clawPivotMotion()
// {
//     double clawPivotGearRatio = 3;

//     double currentLiftAngle = getRotation(lift.get_position()/100);
//     double currentClawAngle = getRotation(claw_pivot_joint.get_position() * clawPivotGearRatio);

//     double power = 0;
//     if (currentLiftAngle >= 180)
//     {
//         double targetPos = -currentLiftAngle * clawPivotGearRatio;
//         power = clawPivotPID.compute(targetPos, currentClawAngle);
//     }
//     else
//     {
//         double targetPos = (-currentLiftAngle * clawPivotGearRatio) + 180;
//         power = clawPivotPID.compute(targetPos, currentClawAngle);
//     }

//     claw_pivot_joint.move(power);
//     pros::delay(20);
// }

// Lift to an angle in degrees. MUST BE IN A [-180,180] RANGE.
void liftToAngle(double targetAngle, int timeout = 9999999)
{
    int t = 0;
    liftPID.reset();

    double currentLiftAngle = getRotation(lift.get_position()/100);

    while (abs(targetAngle - currentLiftAngle) > 1)
    {
        t += 20;
        currentLiftAngle = getRotation(lift.get_position()/100);
        double liftPower = std::clamp(liftPID.compute(targetAngle, currentLiftAngle), -liftClamp, liftClamp);

        lift_mg.move(liftPower);

        if (t >= timeout)
        {
            break;
        }
        pros::delay(20);
    }

    lift_mg.move(0);
}

// // Drive to the object detected so that the lift's claw is positioned over the object.
// // ONLY USE WHEN ALIGNED TO A GOAL OR OBJECT TO BEGIN WITH AND IF THE DISTANCE TO THE OBJECT IS LESS THAN 4FT PREFERABLY.
// void driveLiftOverObject(int timeout = 99999)
// {
//     int t = 0;
//     drivePID.reset();
//     turnPID.reset();

//     double currentLiftAngle = getRotation(lift.get_position()/100);
//     double horizontalLiftDisplacement = getLiftHorizontalDisplacement(liftLength, currentLiftAngle);

//     double currentDistance = mmToInches(distanceForward.get()) + forwardDistanceOffset;
//     double targetDistance = horizontalLiftDisplacement;

//     double targetAngle = getRotation(inertial.get_rotation());

//     while (abs(targetDistance - currentDistance) > 1.5)
//     {
//         t += 20;
//         currentLiftAngle = getRotation(lift.get_position()/100);
//         horizontalLiftDisplacement = getLiftHorizontalDisplacement(liftLength, currentLiftAngle);
//         targetDistance = horizontalLiftDisplacement;
//         currentDistance = mmToInches(distanceForward.get()) + forwardDistanceOffset;
//         double currentAngle = getRotation(inertial.get_rotation());

//         double power = std::clamp(drivePID.compute(targetDistance, currentDistance), -driveClamp, driveClamp);
//         double turnPower = std::clamp(turnPID.compute(targetAngle, currentAngle), -turnClamp, turnClamp);

//         left_mg.move(power - turnPower);
//         right_mg.move(power + turnPower);

//         if (t >= timeout)
//         {
//             break;
//         }

//         pros::delay(20);
//     }

//     left_mg.move(0);
//     right_mg.move(0);
// }
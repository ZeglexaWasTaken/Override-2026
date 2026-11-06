#include "main.h"
#include "util.hpp"

double driveClamp = 127;
double turnClamp = 127;
double liftClamp = 127;

bool clawFaceToggle = true; // True for forwards, False for backwards
bool lifting = false;

pros::Distance distanceLeft(1);
pros::Distance distanceRight(2);
pros::Distance distanceForward(3);

pros::Motor claw_pivot_joint(8);

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

    int targetTicksToPass = 5;
    int ticks = 0;

    while (true)
    {
        t += 20;

        currentAngle = getRotation(inertial.get_rotation());

        double adjustedTarget = currentAngle + getRotation(targetAngle - currentAngle);

        double power = turnPID.compute(adjustedTarget, currentAngle);
        power = std::clamp(power, -turnClamp, turnClamp);

        left_mg.move(power);
        right_mg.move(-power);

        if (t >= timeout)
        {
            break;
        }

        if (abs(getRotation(targetAngle - currentAngle)) <= 5)
        {
            ticks += 1;

            if (ticks >= targetTicksToPass)
            {
                break;
            }
        }
        else {
            ticks = 0;
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
void updateCoordinatesWithEncoders()
{
    double prevForward = 0;
    double prevSide = 0;
    double prevTheta = 0;

    while (true)
    {
    double theta = degToRadians(getRotation(inertial.get_rotation()));
    double thetaDelta = theta - prevTheta;

    double rawSideDeg = (strafingWheel.get_position() - prevSide) / 100.0; 
    double sideDelta = degToInches(rawSideDeg, 2 * M_PI, 1) - (0.5 * thetaDelta);

    double rawForward = (forwardWheel.get_position() - prevForward) / 100.0; 
    double forwardDelta = -degToInches(rawForward, 2 * M_PI, 1) + (0.5 * thetaDelta);

    prevForward = forwardWheel.get_position();
    prevTheta = theta;
    prevSide = strafingWheel.get_position();

    double odomXDelta = (forwardDelta * cos(theta)) + (sideDelta * sin(theta));
    double odomYDelta = (forwardDelta * sin(theta)) - (sideDelta * cos(theta));

    odomY += odomYDelta;
    odomX += odomXDelta;

        
        pros::delay(10);
    }
}

// Drive to point X and Y. Angle is determined automatically to get to destination. Angle after command is finished NOT consistent.
// Do not use this command if there are objects in the way or if turning is too sharp. Use turnToPoint for sharp turning, THEN drive to point.
void driveToPoint(double targetX, double targetY, bool reverse, int timeout = infinity())
{
    drivePID.reset();
    turnPID.reset();
    int t = 0;
    int targetTicksToPass = 8;
    int ticks = 0;
    
    double startX = odomX;
    double startY = odomY;

    double pathX = targetX - startX;
    double pathY = targetY - startY;

    double initialDistance = std::hypot(pathX, pathY);

    double unitX = pathX / initialDistance;
    double unitY = pathY / initialDistance;

    double initialTurnClamp = turnClamp;

    double targetAngle = radToDegrees(atan2(pathY, pathX));
    targetAngle = getRotation(targetAngle);

    while (true)
    {
        t += 20;

        double progress = (odomX - startX) * unitX + (odomY - startY) * unitY;

        double remainingDistance = initialDistance - progress;

        double drivePower = drivePID.compute(0, -remainingDistance);
        drivePower = std::clamp(drivePower, -driveClamp, driveClamp);

        // double targetAngle = radToDegrees(atan2(targetY - odomY, targetX - odomX));
        // targetAngle = getRotation(targetAngle);

        double turnPower = turnPID.compute(targetAngle, getRotation(inertial.get_rotation()));
        turnPower = std::clamp(turnPower, -turnClamp, turnClamp);

        // Switch operator signs if reverse is true.
        left_mg.move((drivePower) + (turnPower));
        right_mg.move((drivePower) - (turnPower));

        if (t >= timeout)
        {
            break;
        }


        if (std::abs(remainingDistance) <= 1.25)
        {
            turnClamp = 10;
            ticks += 1;

            if (ticks >= targetTicksToPass) {break;}
        }
        else {
            ticks = 0;
        }
        pros::delay(20);
    }
    
    left_mg.move(0);
    right_mg.move(0);
    turnClamp = initialTurnClamp;
}

// Drive forward/backwards without following an X and Y coordinate while still maintaining a certain angle.
// ANGLE MUST BE [-180, 180] RANGE
void driveBlindWithAngle(double power, double targetAngle, double duration)
{
    int t = 0;

    while (true)
    {
        t += 20;
        double turnPower = turnPID.compute(targetAngle, getRotation(inertial.get_rotation()));
        turnPower = std::clamp(turnPower, -turnClamp, turnClamp);

        // Switch operator signs if reverse is true.
        left_mg.move((power) + (turnPower));
        right_mg.move((power) - (turnPower));

        if (t >= duration) {break;}

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
    int targetTicksToPass = 2;
    int ticks = 0;

    double currentLiftAngle = getRotation(lift.get_position()/100);
    lifting = true;

    while (true)
    {
        t += 20;
        currentLiftAngle = getRotation(lift.get_position()/100);
        double liftPower = std::clamp(liftPID.compute(targetAngle, currentLiftAngle), -liftClamp, liftClamp);

        lift_mg.move(liftPower);

        if (t >= timeout)
        {
            break;
        }

        if (abs(targetAngle - currentLiftAngle) <= 3)
        {
            ticks += 1;

            if (ticks >= targetTicksToPass) {break;}
        }
        pros::delay(20);
    }

    lift_mg.move(0);
    lifting = false;
}

// Hold the lift to an angle while the lift isn't being moved. Will stop after a lift is commanded.
void holdLift(int timeout = infinity())
{
    liftPID.reset();

    double currentLiftAngle = getRotation(lift.get_position()/100);
    double targetAngle = currentLiftAngle;

    lifting = false;
    int t = 0;

    while (true)
    {
        t += 20;
        currentLiftAngle = getRotation(lift.get_position()/100);
        double liftPower = std::clamp(liftPID.compute(targetAngle, currentLiftAngle), -liftClamp, liftClamp);

        lift_mg.move(liftPower);

        if (lifting) {
            break;
        }

        if (t >= timeout) {
            break;
        }
        pros::delay(20);
    }
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
#pragma once
#include "main.h"

constexpr double wheelDiameter = 2.75;
constexpr double wheelCircumference = wheelDiameter * M_PI;

constexpr double gearRatio = 0.75;

// Important class used to smooth out motion. MUST USE for any autonomous command involving Lift or Drivetrain.
class PID {
    private:
        double kP;
        double kI;
        double kD;

        double error = 0;
        double prevError = 0;

        double integral = 0;
        double derivative = 0;

        double output = 0;
        double integralLimit;

    public:
        PID(double p, double i, double d, double iLimit = 1000) : kP(p), kI(i), kD(d), integralLimit(iLimit) {}

        // P is for Proportional: Power is directly propotional to error(distance to target)
        // I is for Integral: Power is directly proportional to sum of error over time
        // D is for Derivitave: Power is directly proportional to currentError - prevError(errorDelta)
        // Added together these values help to mitigate imperfections on the field with pure math.

        double compute(double target, double current) 
        {
            error = target - current;

            // I often goes out of control due to its power ever increasing, so a limit is necessary incase you set the value too high.
            if (abs(error) < integralLimit) 
            {
                integral += error;
            }
            else 
            {
                integral = 0;
            }

            derivative = error - prevError;
            output = (error * kP) + (integral * kI) + (derivative * kD);
            prevError = error;

            return output;
        }

        void reset() 
        {
            error = 0;
            prevError = 0;
            integral = 0;
            derivative = 0;
            output = 0;
        }
};

double radToDegrees(double rad);
double mmToInches(double mm);
double degToInches(double deg, double circum, double ratio);
double degToRadians(double deg);
int boolToSign(bool b);

PID turnPID(0, 0, 0);
PID drivePID(0, 0, 0);
PID liftPID(0, 0, 0);
PID clawPivotPID(0, 0, 0);

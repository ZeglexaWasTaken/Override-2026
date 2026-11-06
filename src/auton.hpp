#pragma once
double getRotation(double unboundedRot);

void turnToAngle(double targetAngle, int timeout = 1500);

void turnToPoint(double targetX, double targetY, bool reverse, int timeout = 1500);

void updateCoordinatesWithEncoders();

void driveToPoint(double targetX, double targetY, bool reverse, int timeout = 999999999);

void driveLiftOverObject(double timeout = 99999);

void driveBlindWithAngle(double power, double targetAngle, double duration);

void liftToAngle(double targetAngle, int timeout = 9999999);
void holdLift(int timeout = infinity());

extern double odomX;
extern double odomY;
extern double driveClamp;
extern double turnClamp;
extern double liftClamp;

extern bool lifting;
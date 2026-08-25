#pragma once
double getRotation(double unboundedRot);

void turnToAngle(double targetAngle, int timeout = 1500);

void turnToPoint(double targetX, double targetY, bool reverse, int timeout = 1500);

void updateCoordinatesWithEncoders();

void driveToPoint(double targetX, double targetY, bool reverse, int timeout = 999999999);

void driveLiftOverObject(double timeout = 99999);

void liftToAngle(double targetAngle);

extern double odomX;
extern double odomY;
extern double driveClamp;
extern double turnClamp;
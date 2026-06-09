#pragma once
double getRotation(double unboundedRot);

void turnToAngle(double targetAngle, int timeout = 1500);

void turnToPoint(double targetX, double targetY, bool reverse, int timeout = 1500);

void updateCoordinatesWithEncoders();

void driveToPoint(double targetX, double targetY, bool reverse, int timeout = 999999999);
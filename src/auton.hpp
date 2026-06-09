#pragma once
double getRotation(double unboundedRot);

void turnToAngle(double targetAngle, int timeout = 1500);

void updateCoordinatesWithEncoders();

void driveToPointForward(double targetX, double targetY, int timeout = 999999999);

void driveToPointBackward(double targetX, double targetY, int timeout = 999999999);
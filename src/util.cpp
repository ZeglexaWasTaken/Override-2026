#pragma once
#include "main.h"


double mmToInches(double mm)
{
    return mm / 25.4;
}

double radToDegrees(double rad)
{
    return rad / (M_PI / 180);
}
double degToRadians(double deg)
{
    return deg * (M_PI / 180);
}

double degToInches(double deg, double circum, double ratio)
{
    return deg * ((circum / 360.0) * ratio);
}

int boolToSign(bool b)
{
    if (b)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}
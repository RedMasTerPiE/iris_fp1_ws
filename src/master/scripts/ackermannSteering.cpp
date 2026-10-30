#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace std;

const double pi = acos(-1);
const double wheelBase = 2.62;
const double maxSteerDeg = 45.0;

double steeringAngle(double wheelBase, double targetX, double targetY, double pi)
{
    double lookaheadDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double alpha = atan2(targetX, targetY);
    double turningRadiusVal = lookaheadDistance/(2*sin(alpha));
    double delta = atan2(wheelBase, turningRadiusVal);

    double deltaDeg = delta * 180/pi;

    return clamp(deltaDeg, -maxSteerDeg, maxSteerDeg);
}

double turningRadius(double wheelBase, double targetX, double targetY) 
{   
    double lookaheadDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double alpha = atan2(targetX, targetY);

    return lookaheadDistance/(2*sin(alpha));
}

int main(int argc, char** argv) 
{   
    if(argc < 5)
    {   
        cerr << "Error: " << argv[0] << " didn't manage to be executed properly, please input your value according to the following format: <x0> <y0> <x> <y>";
        return 1;
    }
    
    double x0 = stof(argv[1]), y0 = stof(argv[2]), x = stof(argv[3]), y = stof(argv[4]);

    double targetX = x - x0;
    double targetY = y - y0;

    double steerAngle = steeringAngle(wheelBase, targetX, targetY, pi);
    double turningR = turningRadius(wheelBase, targetX, targetY);

    return 0;
}
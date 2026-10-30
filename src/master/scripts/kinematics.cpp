#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace std;

const double pi = acos(-1);
const double wheelBase = 2.62;
const double maxSteerDeg = 34.0;
const double maxLateralAcc = 0.5;
const double carSpeed = 2.0;
const double slowDist = 2.0;
const double maxSpeed = 3.73;

double steeringAngle(double wheelBase, double targetX, double targetY)
{
    double lookaheadDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double alpha = atan2(targetX, targetY);
    double turningRadiusVal = lookaheadDistance/(2*sin(alpha));
    double delta = atan(wheelBase/turningRadiusVal);

    double deltaDeg = delta * 180/pi;

    return clamp(deltaDeg, -maxSteerDeg, maxSteerDeg);
}

double turningRadius(double wheelBase, double targetX, double targetY) 
{   
    double lookaheadDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double alpha = atan2(targetX, targetY);

    return lookaheadDistance/(2*sin(alpha));
}

double speedTarget(double distance, double turnRadius) 
{   
    double vel = carSpeed;

    if(!isinf(turnRadius))
        vel = min(vel, sqrt(maxLateralAcc*fabs(turnRadius)));
    if(distance < slowDist)
        vel = min(vel, carSpeed*distance/slowDist);

    return vel;
}

int speedToPwm(double speed) 
{   
    return static_cast<int>(lround(clamp(speed*255.0/maxSpeed, -255.0, 255.0)));
}

int main(int argc, char** argv) 
{   
    if(argc < 5)
    {   
        cerr << "Error: " << argv[0] << " didn't manage to be executed properly, please input your value according to the following format: <x0> <y0> <x> <y>";
        return 1;
    }
    
    double x0 = stod(argv[1]), y0 = stod(argv[2]), x = stod(argv[3]), y = stod(argv[4]);

    double targetX = x - x0;
    double targetY = y - y0;
    double dist = sqrt(pow(targetX, 2) + pow(targetY, 2));

    double steerAngle = steeringAngle(wheelBase, targetX, targetY);
    double turningR = turningRadius(wheelBase, targetX, targetY);
    double speed = speedTarget(dist, turningR);

    int packetSteer = -static_cast<int>(lround(steerAngle));

    return 0;
}
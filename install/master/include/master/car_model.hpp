// Geometry and driving limits of the IRIS car, in one place.
//
// The values here are the defaults; every node that uses them exposes them
// as a ROS parameter, so they can be retuned without recompiling.
//
// Frame conventions used across the master package:
//   map frame  : x east, y north, theta counter-clockwise from +x
//   body frame : x to the right of the car, y forward
//                (this is the "bird's eye view" the kinematics are written in)
//   steering   : an angle is "+ = left" inside the ROS code; the sign is
//                flipped once, on the wire, by udpSender's steer_wire_sign

#pragma once

namespace car_model {

// --- geometry (metres) -----------------------------------------------------

// Front axle to rear axle. Matches the real car and cmdvel2gazebo.py.
constexpr double kWheelBase = 2.62;

// Left rear wheel to right rear wheel.
constexpr double kTrackWidth = 1.301;

// Rolling radius of the rear wheels.
constexpr double kWheelRadius = 0.356;

// Encoder resolution: ticks per full wheel revolution (gazebo2udp.py).
constexpr int kTicksPerRev = 1024;

// --- driving limits --------------------------------------------------------

// Front steering joint limit is 0.6 rad = 34.4 deg.
constexpr double kMaxSteerDeg = 34.0;

// Speed the PWM scale is normalised against: 255 PWM is this many m/s.
constexpr double kMaxSpeed = 3.73;

// Speed the kinematics aims for on a straight line (m/s).
constexpr double kCruiseSpeed = 2.0;

// Cornering limit: speed is capped so that v^2 / radius stays below this.
constexpr double kMaxLateralAcc = 0.5;

// Below this distance to the target the car slows down to a stop (m).
constexpr double kSlowdownDist = 2.0;

}  // namespace car_model

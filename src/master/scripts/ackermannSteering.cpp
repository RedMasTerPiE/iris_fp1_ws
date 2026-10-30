// Ackermann steering node (package: master).
// Target point in the BEV plane -> turning radius -> steering angle, speed -> PWM,
// published as [pwm, steer_deg] on /drive_command, which udpSender sends to the simulation.
//
// Input : /ackermann/target  geometry_msgs/Point
//           x = sideways (+ right), y = forward (+ ahead), metres
//         parameters car_x, car_y (default 0.0): where the car is in that same plane
// Output: /drive_command     std_msgs/Int32MultiArray  [pwm, steer_deg]   (steer: + = left)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std;

const double pi = acos(-1);
const double wheelBase = 2.62;
const double maxSteerDeg = 34.0;
const double maxLateralAcc = 0.5;
const double carSpeed = 2.0;
const double slowDist = 2.0;
const double maxSpeed = 3.73;

// Radius of the arc to the target. Positive = right, negative = left, infinity = straight.
double turningRadius(double targetX, double targetY)
{
    double lookaheadDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
    if (lookaheadDistance < 1e-6) return INFINITY;     // target is on the car

    double alpha = atan2(targetX, targetY);
    return lookaheadDistance/(2*sin(alpha));           // sin(alpha) = 0 gives infinity: straight
}

double steeringAngle(double turningR)
{
    double delta = atan(wheelBase/turningR);           // atan keeps the sign; atan(0) = 0 for infinity
    double deltaDeg = delta * 180/pi;

    return clamp(deltaDeg, -maxSteerDeg, maxSteerDeg);
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

class AckermannSteering : public rclcpp::Node {
public:
    AckermannSteering() : Node("ackermann_steering")
    {
        x0_ = declare_parameter<double>("car_x", 0.0);
        y0_ = declare_parameter<double>("car_y", 0.0);

        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/car_cmd", 10);

        // optional: targets from other nodes (vision/master) still work
        sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/ackermann/target", 10,
            [this](const geometry_msgs::msg::Point &t) { set_target(t.x, t.y); });

        timer_ = create_wall_timer(chrono::milliseconds(50), [this]() { update(); });  // 20 Hz

        input_thread_ = thread([this]() { read_terminal(); });
        input_thread_.detach();

        RCLCPP_INFO(get_logger(),
                    "Type 'x y' (metres, x = right, y = ahead) and press Enter. "
                    "'stop' releases control back to udpSender's keyboard.");
    }

private:
    void set_target(double x, double y)
    {
        lock_guard<mutex> lock(mtx_);
        target_.x = x;
        target_.y = y;
        have_target_ = true;
    }

    void read_terminal()
    {
        string line;
        while (rclcpp::ok() && getline(cin, line)) {
            if (line == "stop" || line == "s") {
                lock_guard<mutex> lock(mtx_);
                have_target_ = false;
                printf("target cleared\n");
                continue;
            }
            istringstream ss(line);
            double x, y;
            if (ss >> x >> y) {
                set_target(x, y);
                printf("target set to (%.2f, %.2f)\n", x, y);
            } else if (!line.empty()) {
                printf("enter two numbers like: 1 5   (or 'stop')\n");
            }
        }
    }

    void update()
    {
        geometry_msgs::msg::Point t;
        {
            lock_guard<mutex> lock(mtx_);
            if (!have_target_) return;      // silent -> udpSender falls back to keyboard
            t = target_;
        }

        double targetX = t.x - x0_;
        double targetY = t.y - y0_;
        double dist = sqrt(pow(targetX, 2) + pow(targetY, 2));

        double turningR = turningRadius(targetX, targetY);
        double steerAngle = steeringAngle(turningR);
        double speed = speedTarget(dist, turningR);

        int pwm = speedToPwm(speed);
        int packetSteer = -static_cast<int>(lround(steerAngle));   // BEV: + = right, packet: + = left

        std_msgs::msg::Int32MultiArray cmd;
        cmd.data = {pwm, packetSteer};
        pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "target (%.2f, %.2f): radius %.2f m, steering %d deg, pwm %d",
                             t.x, t.y, turningR, packetSteer, pwm);
    }

    double x0_, y0_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mutex mtx_;
    geometry_msgs::msg::Point target_;
    bool have_target_{false};
    thread input_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<AckermannSteering>());
    rclcpp::shutdown();
    return 0;
}
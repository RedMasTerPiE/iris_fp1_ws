#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp> // Added for image encodings
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

using namespace std;
using namespace cv;

class BEV : public rclcpp::Node {
public:
    BEV() : Node("bev") {
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front", 10,
            std::bind(&BEV::imageCallback, this, std::placeholders::_1)); // Explicit placeholders

        // Create a tuning window and trackbars
        namedWindow("Tuning", WINDOW_NORMAL);
        createTrackbar("Top X Width", "Tuning", &top_x_, 100);
        createTrackbar("Top Y Height", "Tuning", &top_y_, 100);
        createTrackbar("Bottom X Width", "Tuning", &bottom_x_, 100);
        createTrackbar("Bottom Y Height", "Tuning", &bottom_y_, 100);

        RCLCPP_INFO(this->get_logger(), "BEV Tuning Node running.");
    }

private:
    int top_x_ = 40;    // Default 40%
    int top_y_ = 60;    // Default 60%
    int bottom_x_ = 0;  // Default 0%
    int bottom_y_ = 100; // Default 100%

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        int width = frame.cols;
        int height = frame.rows;
        
        // Use live trackbar values for percentages
        vector<Point2f> src_pts = {
            Point2f(width * (top_x_ / 100.0f), height * (top_y_ / 100.0f)),             // Top Left
            Point2f(width * ((100 - top_x_) / 100.0f), height * (top_y_ / 100.0f)),     // Top Right
            Point2f(width * ((100 - bottom_x_) / 100.0f), height * (bottom_y_ / 100.0f)), // Bottom Right
            Point2f(width * (bottom_x_ / 100.0f), height * (bottom_y_ / 100.0f))        // Bottom Left
        };

        vector<Point2f> dst_pts = {
            Point2f(0, 0), Point2f(width, 0), 
            Point2f(width, height), Point2f(0, height)
        };

        Mat matrix;
        Mat bev_frame;
        try {
          matrix = getPerspectiveTransform(src_pts, dst_pts);
        } catch (const cv::Exception &e) {
          // Don't crash on a degenerate trapezoid (e.g. flipped trackbars)
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 500,
              "Skipping frame: invalid BEV quad (%s)", e.msg);
          return;
        }
        warpPerspective(frame, bev_frame, matrix, frame.size());

        imshow("Original", frame);
        imshow("BEV", bev_frame);
        waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<BEV>());
    destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

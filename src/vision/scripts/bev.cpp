#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace std;
namespace cv;

class BEV : public rclcpp::Node {
public:
    BEV() : Node("bev") {
        // Ganti "/camera/image_raw" dengan nama topik kamera aktual dari Gazebo simulasi IRIS
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front", 10,
            bind(&BEV::imageCallback, this, placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Vision Node (BEV & Thresholding) berjalan.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        cv_bridge::CvImagePtr cv_ptr;
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        // 1. Definisikan Titik Source (Trapesium pada gambar asli)
        // Nilai (x, y) ini perlu di-tuning (kalibrasi) sesuai posisi dan FOV kamera Gazebo
        int width = frame.cols;
        int height = frame.rows;
        
        vector<Point2f> src_pts = 
        {
            Point2f(width * 0.4f, height * 0.6f), // Kiri Atas
            Point2f(width * 0.6f, height * 0.6f), // Kanan Atas
            Point2f(width * 1.0f, height * 1.0f), // Kanan Bawah
            Point2f(width * 0.0f, height * 1.0f)  // Kiri Bawah
        };

        // 2. Definisikan Titik Destination (Persegi panjang pada gambar BEV)
        vector<Point2f> dst_pts = 
        {
            Point2f(0, 0),                        // Kiri Atas
            Point2f(width, 0),                    // Kanan Atas
            Point2f(width, height),               // Kanan Bawah
            Point2f(0, height)                    // Kiri Bawah
        };

        // 3. Terapkan Warp Perspective (BEV)
        Mat matrix = getPerspectiveTransform(src_pts, dst_pts);
        Mat bev_frame;
        warpPerspective(frame, bev_frame, matrix, frame.size());

        // 4. Thresholding (Contoh: Mengambil warna garis putih)
        Mat gray, thresh;
        cvtColor(bev_frame, gray, COLOR_BGR2GRAY);
        // Mengubah piksel > 200 menjadi putih (255), sisanya hitam (0)
        threshold(gray, thresh, 200, 255, THRESH_BINARY);

        // Menampilkan hasil menggunakan HighGUI OpenCV untuk proses tuning
        imshow("Original", frame);
        imshow("BEV", bev_frame);
        imshow("Thresholding", thresh);
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
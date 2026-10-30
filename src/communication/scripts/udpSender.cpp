#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std;  //lgtm [cpp/using_namespace]

namespace {

constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
constexpr size_t kPacketSize = 12;
constexpr auto kPeriod = chrono::milliseconds(50);        // 20 Hz
constexpr auto kExternalTimeout = chrono::milliseconds(300);

constexpr int32_t kPwmStep = 25;
constexpr int32_t kSteerStep = 10;
constexpr int32_t kPwmMax = 255;
constexpr int32_t kSteerMax = 90;

void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
  pwm = clamp<int32_t>(pwm, -kPwmMax, kPwmMax);
  steer = clamp<int32_t>(steer, -kSteerMax, kSteerMax);
  // htole32 keeps the wire format little-endian on any host
  uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
  uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

  memcpy(buf, kHeader, sizeof(kHeader));
  memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
  memcpy(buf + 8, &steer_le, sizeof(steer_le));
}

}  // namespace

class UdpSender : public rclcpp::Node {
 public:
  UdpSender() : Node("udp_sender") {
    host_ = declare_parameter<string>("host", "127.0.0.1");
    port_ = declare_parameter<int>("port", 12151);
    topic_ = declare_parameter<string>("cmd_topic", "car_cmd");

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw runtime_error("failed to create UDP socket");
    }

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr_.sin_addr) != 1) {
      close(sock_);
      throw runtime_error("invalid host: " + host_);
    }

    // external commands (e.g. from ackermannSteering): data = [pwm, steer_deg]
    sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        topic_, 10,
        [this](std_msgs::msg::Int32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 2) return;
          ext_pwm_ = msg->data[0];
          ext_steer_ = msg->data[1];
          last_ext_ = chrono::steady_clock::now();
          have_ext_ = true;
        });

    enable_raw_mode();

    timer_ = create_wall_timer(kPeriod, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "Sending UDP to %s:%d @ 20 Hz, listening on /%s",
                host_.c_str(), port_, topic_.c_str());
    if (kb_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "WASD: W/S = pwm +/-%d, A/D = steer -/+%d deg, "
                  "SPACE = stop, C = center steer",
                  kPwmStep, kSteerStep);
    }
  }

  ~UdpSender() override {
    restore_terminal();
    // stop the car, keep the current steering angle (like the template)
    uint8_t packet[kPacketSize];
    build_packet(packet, 0, last_steer_sent_);
    sendto(sock_, packet, sizeof(packet), 0,
           reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
    close(sock_);
    printf("Stopped\n");
  }

 private:
  void enable_raw_mode() {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(get_logger(), "stdin is not a TTY, keyboard control disabled");
      return;
    }
    tcgetattr(STDIN_FILENO, &orig_term_);
    termios raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);  // ISIG stays on, so Ctrl-C still works
    raw.c_cc[VMIN] = 0;               // non-blocking read
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    kb_enabled_ = true;
  }

  void restore_terminal() {
    if (kb_enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
      kb_enabled_ = false;
    }
  }

  void poll_keyboard() {
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      switch (c) {
        case 'w': case 'W': kb_pwm_ += kPwmStep; break;
        case 's': case 'S': kb_pwm_ -= kPwmStep; break;
        case 'a': case 'A': kb_steer_ -= kSteerStep; break;
        case 'd': case 'D': kb_steer_ += kSteerStep; break;
        case ' ':           kb_pwm_ = 0; break;
        case 'c': case 'C': kb_steer_ = 0; break;
        default: break;
      }
      kb_pwm_ = clamp<int32_t>(kb_pwm_, -kPwmMax, kPwmMax);
      kb_steer_ = clamp<int32_t>(kb_steer_, -kSteerMax, kSteerMax);
    }
  }

  void tick() {
    if (kb_enabled_) poll_keyboard();

    bool external =
        have_ext_ &&
        (chrono::steady_clock::now() - last_ext_) < kExternalTimeout;

    int32_t pwm = external ? ext_pwm_ : kb_pwm_;
    int32_t steer = external ? ext_steer_ : kb_steer_;
    pwm = clamp<int32_t>(pwm, -kPwmMax, kPwmMax);
    steer = clamp<int32_t>(steer, -kSteerMax, kSteerMax);
    last_steer_sent_ = steer;

    uint8_t packet[kPacketSize];
    build_packet(packet, pwm, steer);
    if (sendto(sock_, packet, sizeof(packet), 0,
               reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_)) < 0) {
      perror("sendto");
    }

    if (pwm != last_pwm_ || steer != last_steer_ || external != last_external_) {
      printf("\r[%s] pwm=%4d steer=%4d   ", external ? "EXT" : "KEY", pwm,
                  steer);
      fflush(stdout);
      last_pwm_ = pwm;
      last_steer_ = steer;
      last_external_ = external;
    }
  }

  // params / socket
  string host_, topic_;
  int port_{12151};
  int sock_{-1};
  sockaddr_in addr_{};

  // ROS
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // keyboard state
  termios orig_term_{};
  bool kb_enabled_{false};
  int32_t kb_pwm_{0}, kb_steer_{0};

  // external state
  bool have_ext_{false};
  int32_t ext_pwm_{0}, ext_steer_{0};
  chrono::steady_clock::time_point last_ext_{};

  // bookkeeping
  int32_t last_steer_sent_{0};
  int32_t last_pwm_{9999}, last_steer_{9999};
  bool last_external_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(make_shared<UdpSender>());
  } catch (const exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
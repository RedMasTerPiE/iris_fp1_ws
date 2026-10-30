// Teleop + autonomous input for the IRIS car, as one ROS 2 node.
//
// Usage: ros2 run communication udpSender [host] [port] [max_pwm] [max_steer_deg]
//   defaults: 127.0.0.1 12151 150 25
//
// Two sources of commands, both sent to the simulation over UDP at 20 Hz:
//   * Keyboard, hold to drive:  W forward   S reverse   A steer left   D steer right
//   * ROS topic /drive_command (std_msgs/Int32MultiArray [pwm, steer_deg]),
//     for example from ackermannSteering
// Who drives: a held WASD key wins; otherwise a fresh /drive_command;
// otherwise nobody (zeros are still sent, as the keep-alive).
//
// Other keys:  Space = stop and switch autonomous OFF   E = autonomous ON
//              Q or Esc = quit
//
// Keyboard events come straight from /dev/input/event* (evdev), so key RELEASE
// is detectable. Without keyboard access the node still works for /drive_command.

#include <arpa/inet.h>
#include <endian.h>
#include <fcntl.h>
#include <linux/input.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std;

namespace {

constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
constexpr size_t kPacketSize = 12;
constexpr auto kPeriod = chrono::milliseconds(50);  // 20 Hz
constexpr int32_t kRampStep = 25;                   // 0 -> 150 pwm in ~300 ms
constexpr double kAutoTimeout = 0.3;                // s: older /drive_command counts as gone

enum class Source { kIdle, kManual, kAuto };

// Who drives this tick: held keys win, then a fresh command from the topic.
Source pick_source(bool manual, bool auto_enabled, bool auto_fresh) {
    if (manual) return Source::kManual;
    if (auto_enabled && auto_fresh) return Source::kAuto;
    return Source::kIdle;
}

const char *source_name(Source s) {
    switch (s) {
        case Source::kManual: return "manual (keyboard)";
        case Source::kAuto:   return "autonomous (/drive_command)";
        default:              return "idle";
    }
}

void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
    pwm = clamp<int32_t>(pwm, -255, 255);
    // htole32 keeps the wire format little-endian on any host
    uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
    uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

    memcpy(buf, kHeader, sizeof(kHeader));
    memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
    memcpy(buf + 8, &steer_le, sizeof(steer_le));
}

// Every /dev/input/event* that is a real keyboard with a W key, claimed
// exclusively so keystrokes go only to us, not the terminal.
vector<int> open_keyboards() {
    vector<int> fds;
    for (int n = 0; n < 64; ++n) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/input/event%d", n);

        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;  // device missing or no permission

        // (a) does it report key events at all? (rules out mice/touchpads)
        unsigned char ev_bits[EV_MAX / 8 + 1] = {};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) < 0 ||
            !(ev_bits[EV_KEY / 8] & (1 << (EV_KEY % 8)))) {
            close(fd);
            continue;
        }

        // (b) does it specifically have a W key? (rules out power buttons)
        unsigned char key_bits[KEY_MAX / 8 + 1] = {};
        if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0 ||
            !(key_bits[KEY_W / 8] & (1 << (KEY_W % 8)))) {
            close(fd);
            continue;
        }

        // (c) grab it: while teleop runs, keys type nothing into the terminal
        if (ioctl(fd, EVIOCGRAB, 1) < 0) {
            close(fd);
            continue;
        }

        fds.push_back(fd);
    }
    return fds;
}

// move `current` toward `target` by at most `step` (soft start/stop ramp)
int32_t ramp_toward(int32_t current, int32_t target, int32_t step) {
    if (current < target) return min(current + step, target);
    if (current > target) return max(current - step, target);
    return current;
}

// The ROS side: remembers the latest /drive_command.
class DriveInput : public rclcpp::Node {
public:
    DriveInput() : Node("udp_sender") {
        sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/drive_command", 10, [this](const std_msgs::msg::Int32MultiArray &m) {
                if (m.data.size() < 2) return;
                pwm_ = m.data[0];
                steer_ = m.data[1];
                last_ = chrono::steady_clock::now();
            });
    }

    bool fresh() const {   // did a command arrive recently?
        return chrono::duration<double>(chrono::steady_clock::now() - last_).count()
               <= kAutoTimeout;
    }
    int32_t pwm() const { return pwm_; }
    int32_t steer() const { return steer_; }

private:
    int32_t pwm_ = 0, steer_ = 0;
    chrono::steady_clock::time_point last_{};
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
};

}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    vector<string> args = rclcpp::remove_ros_arguments(argc, argv);  // our own arguments only
    const char *host = args.size() > 1 ? args[1].c_str() : "127.0.0.1";
    int port = args.size() > 2 ? atoi(args[2].c_str()) : 12151;
    int32_t max_pwm = args.size() > 3 ? atoi(args[3].c_str()) : 150;
    int32_t max_steer = args.size() > 4 ? atoi(args[4].c_str()) : 25;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid host: %s\n", host);
        close(sock);
        return 1;
    }

    vector<int> kb = open_keyboards();
    if (kb.empty()) {
        fprintf(stderr,
                "No keyboard with W/A/S/D keys found: /drive_command only.\n"
                "For WASD: sudo usermod -aG input $USER  (then log out/in), or run with sudo.\n");
    }

    auto node = make_shared<DriveInput>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    printf("Sending to %s:%d at 20 Hz\n", host, port);
    printf("hold W/S = drive, A/D = steer | Space = stop + autonomous off | E = autonomous on | Q or Esc = quit\n");
    printf("max_pwm=%d  max_steer=%d deg (keyboard)\n", max_pwm, max_steer);

    bool held_w = false, held_a = false, held_s = false, held_d = false;
    bool auto_enabled = true;
    bool running = true;
    Source last_source = Source::kIdle;
    int32_t pwm = 0, steer = 0;  // what is being sent right now
    uint8_t packet[kPacketSize];
    auto next_send = chrono::steady_clock::now();

    vector<pollfd> pfds;
    for (int fd : kb) pfds.push_back(pollfd{fd, POLLIN, 0});

    while (running && rclcpp::ok()) {
        // (1) sleep until a key event or the next 20 Hz tick, whichever first
        auto now = chrono::steady_clock::now();
        if (now < next_send) {
            int wait_ms = static_cast<int>(
                chrono::duration_cast<chrono::milliseconds>(next_send - now).count());
            poll(pfds.data(), pfds.size(), wait_ms);
        }

        // (2) drain every pending key event so state is up to date
        for (pollfd &p : pfds) {
            for (;;) {
                input_event ev{};
                ssize_t n = read(p.fd, &ev, sizeof(ev));
                if (n != static_cast<ssize_t>(sizeof(ev))) break;  // queue empty
                if (ev.type != EV_KEY) continue;  // ignore sync/LED bookkeeping
                bool down = ev.value != 0;        // 1=press, 2=repeat, 0=release
                switch (ev.code) {
                    case KEY_W:     held_w = down; break;
                    case KEY_A:     held_a = down; break;
                    case KEY_S:     held_s = down; break;
                    case KEY_D:     held_d = down; break;
                    case KEY_SPACE:
                        if (ev.value == 1) {
                            held_w = held_a = held_s = held_d = false;
                            auto_enabled = false;
                            printf("autonomous OFF (press E to enable)\n");
                        }
                        break;
                    case KEY_E:
                        if (ev.value == 1 && !auto_enabled) {
                            auto_enabled = true;
                            printf("autonomous ON\n");
                        }
                        break;
                    case KEY_Q:
                    case KEY_ESC:   if (down) running = false; break;
                    default: break;
                }
            }
        }

        // (2b) let ROS deliver any /drive_command messages that arrived
        executor.spin_some();

        // (3) pick the source, then targets from it; conflicting keys cancel (W+S -> 0)
        bool manual = held_w || held_a || held_s || held_d;
        Source source = pick_source(manual, auto_enabled, node->fresh());
        if (source != last_source) {
            printf("mode: %s\n", source_name(source));
            last_source = source;
        }

        int32_t pwm_target = 0, steer_target = 0;
        if (source == Source::kManual) {
            if (held_w != held_s) pwm_target = held_w ? max_pwm : -max_pwm;
            if (held_a != held_d) steer_target = held_a ? max_steer : -max_steer;
        } else if (source == Source::kAuto) {
            pwm_target = clamp<int32_t>(node->pwm(), -max_pwm, max_pwm);  // max_pwm is a speed cap
            steer_target = node->steer();   // the controller already limits its own steering
        }

        // (4) soft ramp toward the targets, no jerking
        pwm = ramp_toward(pwm, pwm_target, kRampStep);
        steer = ramp_toward(steer, steer_target, kRampStep);

        // (5) always send, even idle: the receiver stops the car after
        //     0.2 s of silence, so the stream itself is the keep-alive
        build_packet(packet, pwm, steer);
        if (sendto(sock, packet, sizeof(packet), 0,
                   reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            perror("sendto");
        }

        // (6) next tick; if we fell behind, resync instead of bursting
        next_send += kPeriod;
        if (chrono::steady_clock::now() > next_send)
            next_send = chrono::steady_clock::now();
    }

    // last packet: explicit stop, so the car halts now instead of waiting
    // for the receiver's own 0.2 s timeout once we go silent
    build_packet(packet, 0, 0);
    sendto(sock, packet, sizeof(packet), 0,
           reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    printf("Stopped\n");

    for (int fd : kb) close(fd);  // closing releases the keyboard grab
    close(sock);
    rclcpp::shutdown();
    return 0;
}
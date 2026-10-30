#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace std;

namespace {

constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
constexpr size_t kPacketSize = 20;

struct Feedback {
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t vel_x_cms;
    int32_t vel_y_cms;
};

int32_t read_i32_le(const uint8_t *src) {
    uint32_t raw;
    memcpy(&raw, src, sizeof(raw));
    // le32toh keeps the wire format little-endian on any host
    return static_cast<int32_t>(le32toh(raw));
}

bool parse_packet(const uint8_t *buf, size_t len, Feedback &out) {
    if (len < kPacketSize || memcmp(buf, kHeader, sizeof(kHeader)) != 0) {
        return false;
    }
    out.left_ticks = read_i32_le(buf + 4);
    out.right_ticks = read_i32_le(buf + 8);
    out.vel_x_cms = read_i32_le(buf + 12);
    out.vel_y_cms = read_i32_le(buf + 16);
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    int port = argc > 1 ? atoi(argv[1]) : 12152;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return 1;
    }

    // Ctrl-C simply kills the process, even while blocked in recvfrom below
    printf("Listening for feedback on UDP port %d (Ctrl-C to stop)\n", port);

    uint8_t buf[64];
    while (true) {
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n < 0) {
            perror("recvfrom");
            continue;
        }

        Feedback fb;
        if (!parse_packet(buf, static_cast<size_t>(n), fb)) {
            fprintf(stderr, "Ignoring malformed packet (%zd bytes)\n", n);
            continue;
        }
        printf("enc L=%d R=%d  vel x=%d y=%d cm/s\n",
                    fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms);
    }
}
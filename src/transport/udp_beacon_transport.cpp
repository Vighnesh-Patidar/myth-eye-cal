#include "mec/transport/udp_beacon_transport.h"

#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mec {
namespace {

#pragma pack(push, 1)
struct Packet {
    uint32_t magic;
    uint64_t sender;
    uint8_t  los_state;
    float    spx, spy, spz;
    uint8_t  payload[128];
};
#pragma pack(pop)
static_assert(sizeof(Packet) == 4 + 8 + 1 + 12 + 128, "unexpected Packet packing");

} // namespace

UdpBeaconTransport::~UdpBeaconTransport() { stop(); }

bool UdpBeaconTransport::start(uint16_t listen_port, uint64_t self_node_id,
                               const char* dest, uint16_t dest_port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    int yes = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
    ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listen_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) { stop(); return false; }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    self_ = self_node_id;
    listen_port_ = listen_port;
    dest_port_ = dest_port ? dest_port : listen_port;
    if (::inet_pton(AF_INET, dest, &dest_addr_) != 1) dest_addr_ = htonl(INADDR_BROADCAST);
    return true;
}

void UdpBeaconTransport::stop() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    listen_port_ = 0;
}

void UdpBeaconTransport::broadcast(const std::array<uint8_t, 128>& payload, uint8_t los_state,
                                   float spx, float spy, float spz) {
    if (fd_ < 0) return;
    Packet p{};
    p.magic = kUdpBeaconMagic;
    p.sender = self_;
    p.los_state = los_state;
    p.spx = spx; p.spy = spy; p.spz = spz;
    std::memcpy(p.payload, payload.data(), 128);

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(dest_port_);
    dst.sin_addr.s_addr = dest_addr_;
    ::sendto(fd_, &p, sizeof(p), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
}

std::vector<BeaconObservation> UdpBeaconTransport::poll() {
    std::vector<BeaconObservation> out;
    if (fd_ < 0) return out;
    Packet p;
    for (;;) {
        const ssize_t n = ::recv(fd_, &p, sizeof(p), 0);
        if (n != static_cast<ssize_t>(sizeof(p))) break; // EAGAIN or short/foreign packet
        if (p.magic != kUdpBeaconMagic) continue;
        if (p.sender == self_) continue; // our own broadcast echoed back
        BeaconObservation o;
        o.sender = p.sender;
        o.los_state = p.los_state;
        o.spx = p.spx; o.spy = p.spy; o.spz = p.spz;
        std::memcpy(o.payload.data(), p.payload, 128);
        out.push_back(o);
    }
    return out;
}

} // namespace mec

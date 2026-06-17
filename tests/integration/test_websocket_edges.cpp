// Edge-case coverage for WebSocketRenderServer beyond the happy path in
// test_websocket_server.cpp:
//   - a non-WebSocket HTTP request is rejected and reaped
//   - a handshake split across two TCP sends still completes
//   - two clients both receive a broadcast (client_count, fan-out)
//   - a >125-byte payload uses the 16-bit extended length and round-trips

#include "mec/render/websocket_render_server.h"
#include "../unit/test_util.h"

#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using namespace mec;

static int connect_client(uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    timeval tv{2, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return -1;
    return fd;
}

static void pump(WebSocketRenderServer& s, int times = 20) {
    for (int i = 0; i < times; ++i) s.poll_events(5);
}

static std::string read_http(int fd) {
    std::string s; char c;
    while (s.find("\r\n\r\n") == std::string::npos) {
        if (::recv(fd, &c, 1, 0) <= 0) break;
        s.push_back(c);
    }
    return s;
}

static bool read_exact(int fd, char* buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        const ssize_t r = ::recv(fd, buf + got, n - got, 0);
        if (r <= 0) return false;
        got += static_cast<size_t>(r);
    }
    return true;
}

// Read one server->client text frame's payload (handles 7-bit and 16-bit len).
static bool read_frame_payload(int fd, std::string& out) {
    char h[2];
    if (!read_exact(fd, h, 2)) return false;
    CHECK((static_cast<uint8_t>(h[0]) & 0x0F) == 0x1); // text
    size_t len = static_cast<uint8_t>(h[1]) & 0x7F;
    if (len == 126) {
        char ext[2];
        if (!read_exact(fd, ext, 2)) return false;
        len = (static_cast<uint8_t>(ext[0]) << 8) | static_cast<uint8_t>(ext[1]);
    }
    out.resize(len);
    return read_exact(fd, &out[0], len);
}

static void test_non_ws_request_rejected() {
    WebSocketRenderServer server;
    CHECK(server.start(0, "127.0.0.1"));
    const int fd = connect_client(server.port());
    CHECK(fd >= 0);
    // Plain HTTP GET, no Sec-WebSocket-Key -> not a valid upgrade.
    const std::string req = "GET / HTTP/1.1\r\nHost: x\r\n\r\n";
    CHECK(::send(fd, req.data(), req.size(), 0) == static_cast<ssize_t>(req.size()));
    pump(server, 30);
    CHECK(server.client_count() == 0); // never handshaked; reaped
    ::close(fd);
    server.stop();
}

static void test_fragmented_handshake() {
    WebSocketRenderServer server;
    CHECK(server.start(0, "127.0.0.1"));
    const int fd = connect_client(server.port());
    CHECK(fd >= 0);

    const std::string p1 =
        "GET /pose HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n";
    const std::string p2 =
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    CHECK(::send(fd, p1.data(), p1.size(), 0) == static_cast<ssize_t>(p1.size()));
    pump(server, 10);
    CHECK(server.client_count() == 0); // header incomplete: must wait
    CHECK(::send(fd, p2.data(), p2.size(), 0) == static_cast<ssize_t>(p2.size()));
    for (int i = 0; i < 50 && server.client_count() == 0; ++i) server.poll_events(10);
    CHECK(server.client_count() == 1);
    const std::string resp = read_http(fd);
    CHECK(resp.find("101") != std::string::npos);
    ::close(fd);
    server.stop();
}

// Connect+handshake a client; returns its fd with the 101 already drained.
static int connect_ws(WebSocketRenderServer& server) {
    const int fd = connect_client(server.port());
    if (fd < 0) return -1;
    const std::string req =
        "GET /pose HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    ::send(fd, req.data(), req.size(), 0);
    return fd;
}

static void test_two_clients_and_extended_length() {
    WebSocketRenderServer server;
    CHECK(server.start(0, "127.0.0.1"));

    const int c1 = connect_ws(server);
    const int c2 = connect_ws(server);
    CHECK(c1 >= 0 && c2 >= 0);
    for (int i = 0; i < 80 && server.client_count() < 2; ++i) server.poll_events(10);
    CHECK(server.client_count() == 2);
    // Drain both 101 responses.
    CHECK(read_http(c1).find("101") != std::string::npos);
    CHECK(read_http(c2).find("101") != std::string::npos);

    // A >125-byte payload forces the 16-bit extended length path.
    std::string payload = "{\"type\":\"pose_frame\",\"pad\":\"";
    payload.append(300, 'x');
    payload += "\"}";
    CHECK(payload.size() > 125 && payload.size() <= 0xFFFF);

    CHECK(server.broadcast(payload) == 2); // fan-out to both

    std::string got1, got2;
    CHECK(read_frame_payload(c1, got1));
    CHECK(read_frame_payload(c2, got2));
    CHECK(got1 == payload);
    CHECK(got2 == payload);

    ::close(c1);
    ::close(c2);
    for (int i = 0; i < 50 && server.client_count() > 0; ++i) server.poll_events(10);
    CHECK(server.client_count() == 0);
    server.stop();
}

int main() {
    test_non_ws_request_rejected();
    test_fragmented_handshake();
    test_two_clients_and_extended_length();
    RUN_TESTS_RETURN();
}
